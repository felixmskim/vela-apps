// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
 *
 * reri-inject - inject a RISC-V RERI error record through /dev/mem.
 *
 *
 * Why not just call devmem
 * ------------------------
 * An injection is three register writes that have to happen in order, and
 * the shell version spawns a separate process for each one, so each opens
 * /dev/mem and mmaps the bank again. Beyond the waste, two things are wrong
 * with it:
 *
 *   - devmem is a rootfs accident. busybox bundles it, so the buildroot
 *     guest has it and Ubuntu does not; on Ubuntu it is reachable only as
 *     "busybox devmem", and devmem2 spells its arguments differently again.
 *
 *   - a dropped write looks like a successful one. The emulator discards a
 *     status_i write unless exactly one of ce/de/ue is set, and silently
 *     keeps the old value if the previous error was never consumed
 *     (riscv_reri_helper.c, riscv_reri_write_status). devmem reports
 *     success either way and nothing happens when the trigger fires.
 *
 * This reads status_i back and refuses to fire when the value did not take.
 *
 *
 * Register layout - qemu/include/hw/riscv/riscv_reri_regs.h
 * ---------------------------------------------------------
 *   hart N bank     = base + 0x1000 * N        (RISCV_RERI_DEV_SIZE)
 *   bank header     = 64 bytes
 *   error record    = 64 bytes, immediately after the header
 *
 *     +0   control_i    arm here; bits 47:32 are eid
 *     +4   eid low byte writing this fires the injection - always last
 *     +8   status_i     what kind of error
 *     +16  addr_i       physical address, memory-class errors only
 *
 * status_i bits: 31:24 ec, 15:12 at, 10:8 tt, 3 ue, 2 de, 1 ce, 0 v.
 * The valid bit is set by hardware and masked out of guest writes.
 *
 * Record indexes above 0 are aliased in the emulator: riscv_reri_write()
 * resolves the record with records[addr / 32] while records are 64 bytes
 * apart, so guest record i lands on internal record 2i. Record 0, which is
 * what the test procedure uses, is unaffected.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define RERI_BASE_DEFAULT	0x4010000UL
#define RERI_HART_STRIDE	0x1000UL
#define RERI_BANK_HEADER	64UL
#define RERI_RECORD_SIZE	64UL

#define REC_CONTROL		0
#define REC_EID			4
#define REC_STATUS		8
#define REC_ADDR		16

/* control_i: ele=1, ces=udes=uues=2. sinv clears a stale valid bit. */
#define CTRL_ARM		0x2a1UL
#define CTRL_SINV		(1UL << 2)

#define STS_CE			(1UL << 1)
#define STS_DE			(1UL << 2)
#define STS_UE			(1UL << 3)

/* Guest-writable bits of status_i, RERI_STS_MASK in the emulator. */
#define STS_WRITE_MASK		0x7800FF3FFFFEUL

static const char *prog = "reri-inject";

static void usage(FILE *f)
{
	fprintf(f,
"Usage: reri-inject [options]\n"
"\n"
"Inject an error into one RERI error record.\n"
"\n"
"  -e CODE     RERI error code                     (default 17)\n"
"  -s SEV      ce | de | ue                        (default ce)\n"
"  -t TT       transaction type 0-7                (default 7, implicit write)\n"
"  -A AT       address type 0-15                   (default 1)\n"
"  -a ADDR     physical address for addr_i, memory-class errors only\n"
"  -H HART     hart index                          (default 0)\n"
"  -r RECORD   error record index                  (default 0)\n"
"  -B BASE     RERI bank base address              (default 0x4010000)\n"
"  -n COUNT    number of injections                (default 1)\n"
"  -i SECONDS  delay between injections            (default 0)\n"
"  -f          invalidate a stale status_i before arming\n"
"  -N          print what would be written, touch nothing\n"
"  -v          verbose\n"
"  -h          this help\n"
"\n"
"RERI error codes that reach the kernel as a processor error:\n"
"\n"
"   9  TLB / page-walk cache data    -> CPER error_type 0x02, TLB error\n"
"  12  hart state error              -> CPER error_type 0x08\n"
"  16  internal watchdog error       -> CPER error_type 0x08\n"
"  17  execution unit / datapath     -> CPER error_type 0x08\n"
"  20  system memory data            -> CPER error_type 0x08\n"
"\n"
"Codes 12, 14-17 and 19-25 all fold onto 0x08 in firmware, so the kernel log\n"
"cannot tell them apart. -v reads the code back from the register.\n");
}

static uint64_t status_value(unsigned ec, unsigned at, unsigned tt, uint64_t sev)
{
	return ((uint64_t)ec << 24) | ((uint64_t)at << 12) |
	       ((uint64_t)tt << 8) | sev;
}

static void stamp(void)
{
	char buf[32];
	time_t t = time(NULL);
	struct tm tm;

	if (localtime_r(&t, &tm) && strftime(buf, sizeof(buf), "%H:%M:%S", &tm))
		printf("[%s] ", buf);
}

int main(int argc, char **argv)
{
	unsigned ec = 17, at = 1, tt = 7, hart = 0, record = 0;
	unsigned long count = 1, interval = 0;
	uint64_t sev = STS_CE, base = RERI_BASE_DEFAULT, paddr = 0;
	int have_addr = 0, force = 0, dry = 0, verbose = 0;
	const char *sevname = "ce";

	unsigned long bank_off, rec_off;
	volatile uint8_t *bank = MAP_FAILED, *rec;
	uint64_t sts, sts_rb;
	unsigned long i;
	int fd = -1, opt, rc = 0;

	if (argc > 0 && argv[0])
		prog = argv[0];

	/*
	 * The test procedure pipes this through tee to keep a record of when each
	 * injection fired. stdout is fully buffered when it is a pipe, so without
	 * this the file stays empty until the process exits - which, for
	 * `-n 4 -i 720`, is thirty-six minutes after the first injection.
	 */
	setvbuf(stdout, NULL, _IOLBF, 0);

	while ((opt = getopt(argc, argv, "e:s:t:A:a:H:r:B:n:i:fNvh")) != -1) {
		switch (opt) {
		case 'e': ec = strtoul(optarg, NULL, 0); break;
		case 't': tt = strtoul(optarg, NULL, 0); break;
		case 'A': at = strtoul(optarg, NULL, 0); break;
		case 'H': hart = strtoul(optarg, NULL, 0); break;
		case 'r': record = strtoul(optarg, NULL, 0); break;
		case 'B': base = strtoull(optarg, NULL, 0); break;
		case 'n': count = strtoul(optarg, NULL, 0); break;
		case 'i': interval = strtoul(optarg, NULL, 0); break;
		case 'a': paddr = strtoull(optarg, NULL, 0); have_addr = 1; break;
		case 'f': force = 1; break;
		case 'N': dry = 1; break;
		case 'v': verbose = 1; break;
		case 'h': usage(stdout); return 0;
		case 's':
			sevname = optarg;
			if (!strcmp(optarg, "ce"))      sev = STS_CE;
			else if (!strcmp(optarg, "de")) sev = STS_DE;
			else if (!strcmp(optarg, "ue")) sev = STS_UE;
			else {
				fprintf(stderr, "%s: severity must be ce, de or ue (got '%s')\n",
					prog, optarg);
				return 2;
			}
			break;
		default:
			usage(stderr);
			return 2;
		}
	}

	if (ec > 255) {
		fprintf(stderr, "%s: RERI error code out of range: %u\n", prog, ec);
		return 2;
	}
	if (tt > 7) {
		fprintf(stderr, "%s: transaction type must be 0-7 (got %u)\n", prog, tt);
		return 2;
	}
	if (at > 15) {
		fprintf(stderr, "%s: address type must be 0-15 (got %u)\n", prog, at);
		return 2;
	}
	if (count < 1) {
		fprintf(stderr, "%s: -n must be at least 1\n", prog);
		return 2;
	}
	if (record > 0)
		fprintf(stderr,
			"%s: warning: record %u is aliased in the emulator, only record 0 is reliable\n",
			prog, record);

	/*
	 * Exactly one of ce/de/ue is set by construction, which is what
	 * riscv_reri_write_status() demands before it accepts the write.
	 */
	sts = status_value(ec, at, tt, sev);

	bank_off = base + RERI_HART_STRIDE * hart;
	rec_off  = RERI_BANK_HEADER + RERI_RECORD_SIZE * record;

	if (verbose || dry) {
		printf("hart %u record %u, RERI code %u, severity %s, tt %u, at %u\n",
		       hart, record, ec, sevname, tt, at);
		printf("  control_i 0x%lx <- 0x%lx%s\n",
		       bank_off + rec_off + REC_CONTROL,
		       CTRL_ARM | (force ? CTRL_SINV : 0),
		       force ? " (with sinv)" : "");
		printf("  status_i  0x%lx <- 0x%lx\n",
		       bank_off + rec_off + REC_STATUS, sts);
		if (have_addr)
			printf("  addr_i    0x%lx <- 0x%lx\n",
			       bank_off + rec_off + REC_ADDR, paddr);
		printf("  eid       0x%lx <- 1   (trigger)\n",
		       bank_off + rec_off + REC_EID);
	}

	if (dry)
		return 0;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "%s: /dev/mem: %s\n", prog, strerror(errno));
		if (errno == EACCES || errno == EPERM)
			fprintf(stderr, "%s: run as root\n", prog);
		return 1;
	}

	bank = mmap(NULL, RERI_HART_STRIDE, PROT_READ | PROT_WRITE, MAP_SHARED,
		    fd, (off_t)bank_off);
	if (bank == MAP_FAILED) {
		fprintf(stderr, "%s: mmap 0x%lx: %s\n", prog, bank_off, strerror(errno));
		fprintf(stderr, "%s: is the guest running with -M virt,reri=true?\n", prog);
		close(fd);
		return 1;
	}
	rec = bank + rec_off;

	for (i = 0; i < count; i++) {
		if (i > 0 && interval) {
			if (verbose)
				printf("waiting %lus\n", interval);
			sleep(interval);
		}

		/*
		 * A pending error that was never consumed makes the emulator
		 * abandon the next injection (riscv_error_injection_tick
		 * checks status_i.v), and the status write below would be
		 * folded into the old record rather than replacing it.
		 */
		if (*(volatile uint64_t *)(rec + REC_STATUS) & 1) {
			if (!force) {
				fprintf(stderr,
					"%s: status_i still valid - the previous error was not consumed.\n",
					prog);
				fprintf(stderr,
					"%s: -f clears it, but check that GHES is alive first.\n",
					prog);
				rc = 1;
				break;
			}
			if (verbose)
				printf("clearing a stale status_i\n");
		}

		*(volatile uint64_t *)(rec + REC_CONTROL) =
			CTRL_ARM | (force ? CTRL_SINV : 0);
		*(volatile uint64_t *)(rec + REC_STATUS) = sts;
		__sync_synchronize();

		/* Did it take? A dropped write is the failure devmem cannot see. */
		sts_rb = *(volatile uint64_t *)(rec + REC_STATUS);
		if ((sts_rb & STS_WRITE_MASK) != (sts & STS_WRITE_MASK)) {
			fprintf(stderr,
				"%s: status_i write was rejected: wrote 0x%lx, read back 0x%lx\n",
				prog, sts, sts_rb);
			rc = 1;
			break;
		}

		if (have_addr) {
			*(volatile uint64_t *)(rec + REC_ADDR) = paddr;
			__sync_synchronize();
		}

		/* The trigger. Always last. */
		*(volatile uint8_t *)(rec + REC_EID) = 1;
		__sync_synchronize();

		stamp();
		printf("injection %lu/%lu fired - RERI code %u, %s\n",
		       i + 1, count, ec, sevname);

		if (verbose)
			printf("  status_i reads back 0x%lx (bits 31:24 are the RERI code)\n",
			       *(volatile uint64_t *)(rec + REC_STATUS));
	}

	munmap((void *)bank, RERI_HART_STRIDE);
	close(fd);
	return rc;
}
