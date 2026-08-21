# RAS — Error Injection and Collection

Tools for exercising the RAS path on a Q-Vela guest: **inject** a hardware error
into the emulated RERI registers, then **watch** the kernel detect it and
**collect** it into a log.

```
    devmem                    kernel                    rasdaemon
  inject an error   ────►   detects, reports   ────►   collects, logs
```

Everything below runs **inside the guest**. Start it first.

---

## Boot the guest

Launching is q-vela's job — the command lives in
[q-vela `docs/ras.md`](https://github.com/riscv-vela/q-vela/blob/main/docs/ras.md)
§3 and that stays the source of truth. This section only says what the RAS flow
needs and where each piece comes from.

### 1. What you need, and who builds it

| Artifact | Repo | How |
|---|---|---|
| `qemu/build/qemu-system-riscv64` | [q-vela](https://github.com/riscv-vela/q-vela) | `docs/ras.md` §2.3. Check `-M virt,help \| grep reri`. |
| `opensbi/.../fw_dynamic.bin` | [vela-fws](https://github.com/riscv-vela/vela-fws) | README — build `sm` first, then OpenSBI |
| `Build/.../RISCV_VIRT_{CODE,VARS}.fd` | vela-fws | README — EDK2, then `truncate -s 32M` **both** |
| `linux/arch/riscv/boot/Image` | [vela-linux](https://github.com/riscv-vela/vela-linux) | needs `CONFIG_RAS=y` — [PR #2](https://github.com/riscv-vela/vela-linux/pull/2) |
| `buildroot/output/images/rootfs.cpio` | vela-fws | README — buildroot |
| `ubuntu-noble-riscv64.img` | [vela](https://github.com/riscv-vela/vela) | `./init.sh` |

### 2. Which guest

| Guest | Good for | rasdaemon? |
|-------|----------|------------|
| **`rootfs.cpio`** | Smoke test — is the RERI path alive? | No, and it does not need one. busybox already ships `devmem`. |
| **Ubuntu 24.04** | Collection, logging, ROS 2 workloads | Yes — `rasdaemon/install-rasdaemon.sh`. |

### 3. Launch, from `q-vela/`

Three flags decide whether any RAS event exists at all: `reri=true` (the MMIO
device), the EDK2 pflash pair (GHES comes from ACPI — a device-tree boot has
none), and `fw_dynamic.bin` rather than `fw_jump.bin`. `q-vela/run.sh` and
`vela/run.sh` have none of them and will not work here.

> **`wg=on` is also required**, and `docs/ras.md` §3 does not list it. OpenSBI
> links the Vela security monitor, which touches the WorldGuard CSR `mlwid`
> (`0x390`) during init; without it the guest dies in M-mode before EDK2 runs:
> `Failed to access CSR 0x390 from M-mode`, `mcause=0x2 mtval=0x39051073`.
> The fix belongs in `docs/ras.md`, not here — raising it with q-vela.

Paths below assume the repositories sit side by side.

```bash
cd q-vela
qemu/build/qemu-system-riscv64 \
  -nographic -accel tcg -m 4096 -smp 2 \
  -bios ../vela-fws/opensbi/build/platform/generic/firmware/fw_dynamic.bin \
  -blockdev node-name=pflash0,driver=file,read-only=on,filename=../vela-fws/Build/RiscVVirtQemu/RELEASE_GCC5/FV/RISCV_VIRT_CODE.fd \
  -blockdev node-name=pflash1,driver=file,filename=../vela-fws/Build/RiscVVirtQemu/RELEASE_GCC5/FV/RISCV_VIRT_VARS.fd \
  -M virt,pflash0=pflash0,pflash1=pflash1,wg=on,rpmi=true,reri=true,aia=aplic-imsic \
  -netdev user,id=net0,hostfwd=tcp::2222-:22 -device virtio-net-device,netdev=net0 \
  -kernel ../vela-linux/linux/arch/riscv/boot/Image \
  -semihosting-config enable=on,userspace=on \
  -initrd ../vela-fws/buildroot/output/images/rootfs.cpio \
  -append "root=/dev/ram rw console=ttyS0 earlycon=uart8250,mmio,0x10000000"
```

For the Ubuntu guest, replace the last two lines with:

```bash
  -drive file=../vela/ubuntu-noble-riscv64.img,format=raw,if=virtio \
  -append "root=/dev/vda1 rw console=ttyS0 earlycon=uart8250,mmio,0x10000000"
```

UEFI writes its variables back into `RISCV_VIRT_VARS.fd`. Copy it somewhere
writable and point `pflash1` at the copy if you want the build artifact left
alone.

A healthy boot shows the chain coming up:

```
sse: SBI SSE extension detected
GHES: Registered SSE notification on vector 32768
EINJ: Error INJection is initialized.
```

Log in as **`root`**, no password. Quit with **Ctrl-A X**.

---

## Before you start

### 1. Mount tracefs

Nothing does this for you on the buildroot guest. `/sys/kernel/tracing` exists as
an empty mount point until you mount it — so the directory looks like it is
there, but everything inside is missing.

```bash
mount -t tracefs nodev /sys/kernel/tracing
```

On the Ubuntu guest systemd normally mounts it at boot, so it may already be
done. Running the command again is harmless.

### 2. Check that the kernel has `CONFIG_RAS=y`

**Without it, rasdaemon collects nothing** — not because it is misconfigured,
but because the tracepoint it listens on is not compiled into the kernel.

`dmesg` will still print `[Hardware Error]`, which makes this easy to miss. The
kernel log and the tracepoint are two separate paths.

```bash
ls /sys/kernel/tracing/events/ras/
```

| You see | Meaning |
|---------|---------|
| `non_standard_event/` | Good. Carry on. |
| `events/` is there, but no `ras/` | `CONFIG_RAS=n`. Stop here — fix the kernel first, see [vela-linux](https://github.com/riscv-vela/vela-linux). |
| `No such file or directory` | tracefs is not mounted. Go back to step 1. |

---

## Smoke test (no rasdaemon needed)

This proves the whole chain — QEMU → OpenSBI → GHES → kernel tracepoint — using
nothing but busybox. Run it on either guest.

```bash
# 0. tracefs must be mounted (see above)
mount -t tracefs nodev /sys/kernel/tracing

# 1. turn the tracepoint on and watch it
echo 1 > /sys/kernel/tracing/events/ras/non_standard_event/enable
cat /sys/kernel/tracing/trace_pipe &

# 2. inject a TLB error
devmem 0x4010040 32 0x2a1        # arm the error record
devmem 0x4010048 32 0x9001404    # describe the error
devmem 0x4010044 8 1             # fire

# 3. check the kernel's own report
dmesg | grep -A15 'Hardware Error'
```

### What a successful run looks like

Two things should appear. First the kernel's own report:

```
{1}[Hardware Error]: Hardware error from APEI Generic Hardware Error Source: 1
{1}[Hardware Error]: event severity: recoverable
{1}[Hardware Error]:  Error 0, type: recoverable
{1}[Hardware Error]:   section_type: general processor error
{1}[Hardware Error]:   processor_type: 3, RISCV
{1}[Hardware Error]:   processor_isa: 6, RISCV64
{1}[Hardware Error]:   error_type: 0x02
{1}[Hardware Error]:   TLB error
{1}[Hardware Error]:   operation: 1, data read
{1}[Hardware Error]:   target_address: 0x0000000000000000
```

And then the tracepoint — this is the one rasdaemon reads:

```
devmem-153 [000] dnh.. 414.591022: non_standard_event: severity: 2;
  sec type:adcc7698-b447-db4b-b65e-16f193c4f3db;
  FRU: 00000000-0000-0000-0000-000000000000 ; data len:192;
  raw data:0f 02 00 00 00 00 00 00 03 06 02 01 00 00 00 00 00 00 ... (192 bytes)
```

Reading that record against the kernel's own structures:

| Field | Value | Means |
|-------|-------|-------|
| `severity` | 2 | `GHES_SEV_RECOVERABLE` |
| `sec type` | `adcc7698-b447-…` | `CPER_SEC_PROC_GENERIC` — the bytes are printed little-endian, so it is the `0x9876CCAD-47B4-4bdb-…` GUID from `include/linux/cper.h` |
| raw byte 8 | `03` | `proc_type` — RISCV |
| raw byte 9 | `06` | `proc_isa` — RISCV64 |
| raw byte 10 | `02` | `proc_error_type` — TLB error |
| raw byte 11 | `01` | `operation` — data read |

`target_address` is zero because we never wrote `addr_i`; it reports whatever the
injector put there.

Note the record arrives as **undecoded raw bytes**. That is expected here — see
[Known limitations](#known-limitations).

### If it does not work

| What you see | What it means |
|--------------|---------------|
| `dmesg` shows the error, `trace_pipe` stays silent | The kernel lacks `CONFIG_RAS`. Go back to [Before you start](#before-you-start). |
| Neither appears | The injection did not fire. Check that you wrote `0x4010044` **last**, and that QEMU was started with `-M virt,reri=true`. |

---

## Installing rasdaemon (Ubuntu guest)

```bash
sudo ./rasdaemon/install-rasdaemon.sh
systemctl status rasdaemon
```

That leaves rasdaemon running as a **systemd service** — it survives your
console session ending, restarts if it dies, and comes back after a reboot.
TC.10.007 collects unattended for the better part of an hour, so
`rasdaemon -f -r &` is not enough.

Five things the script does that a plain `configure && make install` does not:

| | Why |
|---|---|
| Builds v0.8.5 from source | The distro package is 0.8.0-2, and rasdaemon picks its events at **configure time**. |
| `--enable-non-standard` | The only event this platform produces, and off by default. Without it the daemon starts cleanly and records nothing. |
| Never `--enable-reri` | `ras:reri_event` does not exist in this kernel. |
| Installs the systemd unit | Upstream generates it but has no install rule — distro packaging always did that step. |
| Drop-in: tracefs + `Restart=always` | Nothing mounts tracefs on a minimal guest, and `Restart=on-abort` misses a clean non-zero exit. |

### The decoder

Our errors arrive as a CPER **Processor Generic Error Section**. `ghes_do_proc()`
has no branch for it, so it reaches user space on `ras:non_standard_event`, and
rasdaemon has no decoder registered for that GUID — it dumps hex.
`rasdaemon/riscv-decoder/` registers one through the same
`register_ns_ev_decoder()` hook the Ampere, HiSilicon, NVIDIA and Yitian
decoders use.

```
without:  raw data:0f 02 00 00 00 00 00 00 03 06 02 01 ...

with:     processor generic error:
           processor_type: 3, RISCV
           processor_isa: 6, RISCV64
           error_type: 0x02 (TLB error)
           operation: 1, data read
           target_address: 0x0000000000000000
```

Strings come from `drivers/firmware/efi/cper.c`, so the record should read the
same as dmesg. If they disagree, the decoder is wrong.

---

## Injecting errors

```bash
make -C inject                       # one file, no dependencies
sudo ./inject/inject_cpu_ce.sh       # corrected CPU error
sudo ./inject/inject_mem.sh -a 0x80000000
```

TC.10.007 asks for four injections twelve minutes apart:

```bash
sudo ./inject/inject_cpu_ce.sh -n 4 -i 720
```

`-N` prints the register writes without touching anything, `-v` reads the RERI
code back afterwards, `-e CODE` picks a different error. `inject/reri-inject -h`
lists the rest.

### Or by hand

An injection is three writes, and the order matters — the third one is the
trigger.

```bash
devmem 0x4010040 32 0x2a1          # arm the error record
devmem 0x4010048 32 0x11001702     # describe the error (table below)
devmem 0x4010044 8  1              # fire
```

Two reasons the scripts exist rather than this loop being the answer:

- **`devmem` is a rootfs accident.** busybox bundles it, so the buildroot guest
  has it and Ubuntu does not. On Ubuntu it is `busybox devmem` after
  `apt-get install busybox`, and `devmem2` spells its arguments differently
  again. `reri-inject` opens `/dev/mem` itself and behaves the same everywhere.
- **A dropped write looks like a successful one.** The emulator discards a
  `status_i` write unless exactly one of `ce`/`de`/`ue` is set, and keeps the
  old value if the previous error was never consumed. `devmem` reports success
  either way and nothing happens when the trigger fires. `reri-inject` reads
  the register back and refuses to fire.

### Available error types

| Name | RERI code | Severity | Register value | What the kernel reports | Observed? |
|------|-----------|----------|----------------|-------------------------|-----------|
| `tlb` | 9 — TLB/page-walk cache data | deferred | `0x09001404` | `error_type: 0x02` TLB error, `operation: 1, data read`, **recoverable** | **yes** |
| `exec-unit` | 17 — execution unit / datapath | corrected | `0x11001702` | `error_type: 0x08` micro-architectural error, `operation: 2, data write`, **corrected** | **yes** |
| `mem-data` | 20 — system memory data | corrected | `0x14001702` | identical to `exec-unit` — see below | **yes** |
| `hart` | 12 — hart state error | corrected | `0x0C001702` | identical to `exec-unit` | q-vela `docs/ras.md` |
| `watchdog` | 16 — internal watchdog | corrected | `0x10001702` | expected: identical to `exec-unit` | not yet |

### ⚠️ The RERI error code does not survive into the log

Codes 12, 17 and 20 were each injected and each produced a **byte-identical**
CPER record — same `error_type: 0x08`, same `operation: 2`, same raw payload
`… 03 06 08 02 …`. Nine more codes fold into the same bucket. This is not a bug
in the injection; it is where the firmware translates RERI into CPER.

CPER's processor error type is a 4-bit mask (`cper.c`, `cper_proc_error_type_strs`),
and `opensbi/lib/utils/ras/reri_drv.c:373-405` folds all 21 RERI codes onto it:

| CPER `error_type` | Meaning | RERI codes folded into it |
|-------------------|---------|---------------------------|
| `0x01` | cache error | 3, 4, 5, 6 |
| `0x02` | TLB error | 9, 10, 11 |
| `0x04` | bus error | 18 |
| `0x08` | micro-architectural error | **12, 14, 15, 16, 17, 19, 20, 21, 22, 23, 24, 25** |
| `0x00` | (none set) | everything else — 0, 1, 2, 7, 8, 13, 26, 27, … |

Two consequences worth knowing before you design a test around this:

- **You cannot tell "code 17" from "code 12" by reading `dmesg` or the rasdaemon
  record.** Twelve codes share `0x08`. Verified: injecting 12, 17 and 20 gave
  three indistinguishable records.
- **Memory errors do not look like memory errors.** Code 20 (system memory data)
  folds into `0x08` as well, so it is reported as a *micro-architectural
  processor* error. Verified by injection.

> **The memory half of this has a fix pending.** Hayong Jeong (@AngryOvO) traced
> it to two firmware issues — `reri_drv.c` never selects the memory section type,
> and OpenSBI's `cper_mem2_sec` struct does not match the UEFI layout Linux parses
> — and submitted `opensbi_mem_cper.patch` upstream. With it, RERI codes 18–21
> become proper CPER Memory Error sections with distinct error types. Codes 12 and
> 17 are unaffected: they are not memory-class, so they stay folded together.
> vela-fws does not carry the patch yet.

**Workaround — read the register back.** QEMU clears only the valid bit when the
error is consumed (`riscv_reri_helper.c:152` sets `status_i.v = 0`), so the `ec`
field survives:

```bash
devmem 0x4010048 32          # bits 31:24 are the RERI error code
```

That distinguishes all codes, but it reads hardware state rather than the log —
it proves what you injected, not what was recorded.

### Severity mapping

`reri_drv.c:335-342` maps the RERI severity bits to CPER severity:

| RERI bit | CPER severity | `dmesg` shows |
|----------|---------------|---------------|
| `ce` | 2 | corrected |
| `de` | 0 | recoverable |
| `ue` | 1 | fatal |
| none | 3 | informational |

Both observations above match this.

---

## Checking the result

```bash
ras-mc-ctl --errors          # decoded summary
ls -l /var/log/ras/          # rasdaemon's own record
dmesg | grep -A15 'Hardware Error'
```

Paste the raw output into the issue or PR — a summary is not evidence.

### Closing the loop

Once it works, boot a kernel **without** `CONFIG_RAS` and inject again. You
should get zero events. That is what turns "we saw some records" into "these
records came from our injection".

---

## RERI register reference

Each hart gets a 4 KiB error bank. Hart 0 starts at `0x0401_0000`, hart 1 at
`0x0401_1000`, and so on.

Inside a bank there is a 64-byte header, then 64-byte error records:

```
record[i].control_i = bank_base + 64 * (i + 1)
record[i].status_i  = control_i + 8
```

So for hart 0, record 0:

| Address | Register | What you write |
|---------|----------|----------------|
| `0x4010040` | `control_i` | `0x2a1` — enable logging, pick the notification mode |
| `0x4010048` | `status_i` | the error description (see encoding below) |
| `0x4010050` | `addr_i` | a physical address — only needed for memory-class errors |
| `0x4010044` | `control_i + 4` (`eid`) | `1` — **this is the trigger, write it last** |

`addr_i` is optional today: every error currently becomes a processor error,
where the address is reported but not acted on. Once the pending memory-section
patch lands it becomes **required** for memory-class codes — `ghes.c:370` only
builds a memory section when the physical address is non-zero.

> ⚠️ The formula in
> [`reri register generator/readme.md`](../reri%20register%20generator/readme.md)
> gives the Status address as `bank + 72*(i+1)`. That happens to be right for
> record 0 (72 = 0x48) but is wrong from record 1 onward — records are 64 bytes
> apart, not 72. Use `control_i + 8`.

### `status_i` encoding

| Bits | Field | Notes |
|------|-------|-------|
| 31:24 | `ec` | RERI error code — the number the test plan refers to |
| 15:12 | `at` | address type |
| 10:8 | `tt` | transaction type: 4 = explicit read, 5 = explicit write, 6 = implicit read, 7 = implicit write |
| 3 | `ue` | uncorrected |
| 2 | `de` | deferred |
| 1 | `ce` | corrected |
| 0 | `v` | valid — hardware sets this; leave it 0 when injecting |

Read `0x0C001702` back with that table: `ec` = 0x0C = 12 (hart state error),
`at` = 1, `tt` = 7 (implicit write), `ce` = 1 (corrected).

The error code numbers come from `RERI_EC_*` in
`qemu/include/hw/riscv/riscv_reri_regs.h`, and they are the **same numbering the
test plan uses** when it says "RERI Error Code 17".

---

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `ls /sys/kernel/tracing/events/ras/` → `No such file or directory` | tracefs is not mounted. `/sys/kernel/tracing` is only an empty mount point until you run `mount -t tracefs nodev /sys/kernel/tracing`. This is the usual first stumble on a minimal guest. |
| `events/` exists but there is no `ras/` in it | Kernel built without `CONFIG_RAS`. |
| `dmesg` shows the error but `trace_pipe` stays silent | Also `CONFIG_RAS` — the kernel log path works without it, the tracepoint does not. |
| rasdaemon exits with `Can't locate a mounted debugfs` | Neither tracefs nor debugfs is mounted. The service drop-in mounts tracefs before starting; if you launched rasdaemon by hand, mount it yourself. |
| `systemctl start rasdaemon` fails immediately | `journalctl -u rasdaemon -n 30`. Two usual causes: `/etc/sysconfig/rasdaemon` is missing (the unit's `EnvironmentFile=` has no leading dash, so a missing file is fatal), or the tracefs `ExecStartPre` failed. |
| rasdaemon runs but records nothing | Built without `--enable-non-standard`. The install script asserts `HAVE_NON_STANDARD` in `config.h` and stops if it is absent, so this means something else built the binary. |
| Records show `raw data:` hex instead of decoded fields | Built without the RISC-V decoder. The install script asserts `HAVE_RISCV_NS_DECODE` the same way. |
| `devmem` writes succeed, nothing happens at all | Did you write `0x4010044` (the trigger) **last**? `reri-inject` orders them for you and reads the register back. |
| `devmem: command not found` | On the host: these commands run **inside the guest**. In the Ubuntu guest: `apt-get install busybox`, then `busybox devmem` — or use `reri-inject`. |
| Guest dies in M-mode with `Failed to access CSR 0x390` | Booted without `wg=on`. OpenSBI links the security monitor, which needs WorldGuard. |
| `ras-mc-ctl` says `Can't locate DBI.pm` | `apt-get install libdbd-sqlite3-perl`. The daemon is unaffected — this only blocks reading what it collected. |
| No `[Hardware Error]` in `dmesg` at all | The guest must boot through EDK2/ACPI. A device-tree boot produces no GHES reports. |

---

## Background — why it is built this way

<details>
<summary>Why <code>--enable-non-standard</code>, and not <code>--enable-reri</code></summary>

rasdaemon 0.8.5 does have a `--enable-reri` option, and at first glance it looks
like exactly what a RISC-V RERI platform wants. It is not usable here.

`--enable-reri` makes rasdaemon subscribe to the kernel tracepoint
`ras:reri_event`. **Our kernel does not define that tracepoint.** The tracepoints
in `include/ras/ras_event.h` are `extlog_mem_event`, `mc_event`, `arm_event`,
`non_standard_event`, `aer_event` and `memory_failure_event` — that is the whole
list. Building with `--enable-reri` would subscribe to nothing.

What actually happens to a RERI error is this:

1. OpenSBI reports it to the OS through SBI SSE, and it arrives at GHES as a
   CPER **Generic Processor Error** section.
2. `ghes_do_proc()` has branches for platform memory, PCIe, ARM and CXL sections
   — but none for generic processor errors. So it falls through to the final
   `else`, which calls `log_non_standard_event()`.
3. That function emits `ras:non_standard_event` — and it is compiled in only
   when `CONFIG_RAS=y` (`include/linux/ras.h`, `drivers/ras/Makefile`).

So `ras:non_standard_event` is the only event this platform produces, and
`--enable-non-standard` is the only way rasdaemon will listen for it. Without
that flag rasdaemon does not merely ignore the event — the handler is not
compiled in at all (`ras-events.c`, `Makefile.am`, both guarded on
`HAVE_NON_STANDARD`).

If a `ras:reri_event` tracepoint is added to the kernel later, revisit this.
</details>

<details>
<summary>Why there is no buildroot package here</summary>

The `rootfs.cpio` guest is built by buildroot, which lives in
[vela-fws](https://github.com/riscv-vela/vela-fws) — a different repository.
Shipping a buildroot package from *this* repository would mean vela-fws's build
reaching across into vela-apps for files, which is exactly the coupling the
repository split was meant to remove.

It would also not buy much. The `rootfs.cpio` guest is the smoke-test
environment, and the smoke test needs only `devmem`, which busybox already
provides. The test plan's OS is Ubuntu 24.04, and the workloads it runs (Nav2,
SLAM, AI inference) cannot run on a busybox initramfs anyway.

So: `rootfs.cpio` for the quick check, Ubuntu for collection and for the real
test runs.

If the `rootfs.cpio` guest ever does need rasdaemon, the clean way is to bump
vela-fws's buildroot submodule past `7b8892e900` (2025-12-19), which is when
buildroot gained an in-tree `package/rasdaemon`. The currently pinned commit
(`d8b0fdb71e`, 2025-11-09) predates it, which is why `BR2_PACKAGE_RASDAEMON`
does not appear in `menuconfig` today.
</details>

---

## Known limitations

**Records are stored raw, not decoded.** The section GUID that arrives here is
`adcc7698-b447-db4b-b65e-16f193c4f3db` — the CPER *Generic Processor Error*
GUID, confirmed by observation. rasdaemon's non-standard decoders are all for
vendor-specific GUIDs (HiSilicon, Ampere, NVIDIA and so on), so it has nothing
to match and stores the payload as raw bytes.

That is enough to prove detection and collection, and the bytes are readable by
hand against `struct cper_sec_proc_generic` (see the table in
[What a successful run looks like](#what-a-successful-run-looks-like)). It is not
a human-readable fault report. Producing one would mean teaching rasdaemon to
decode this GUID.

**Memory errors go through the hart bank too.** QEMU declares a separate DRAM
error bank at `0x4000000`, but no device is actually created for it — it exists
only as a device-tree node, and this platform boots via ACPI, so nothing is
listening there. Memory-class error codes are therefore written into a hart error
record, the same way CPU ones are.

**EDAC counters are not populated — and a CPU error can never populate them.**
`ce_count` belongs to a *memory controller*, and there is exactly one path to it:

```
ghes_do_proc()   if (guid_equal(sec_type, &CPER_SEC_PLATFORM_MEM))
  → ghes_report_chain → ghes_edac_report_mem_error() → edac_raw_mc_handle_error()
    → dimm->ce_count += e->error_count
```

Two things follow. Today no Platform Memory Error section is produced at all, so
`mc0` never appears — the pending firmware patch above fixes that for codes
18–21. But even then, a CPU-class code such as 17 will never reach this path,
because it is not a memory error. Any test that injects code 17 and expects
`ce_count` to increment is asking for two incompatible things.

**The recorded event cannot identify which RERI code was injected.** See
[the warning above](#-the-reri-error-code-does-not-survive-into-the-log). Any
test whose pass criterion is "the log contains N records of RERI error code X"
needs either a firmware change to carry the code into CPER, or a criterion
rewritten around what the record actually holds.

---

## See also

- [`workload-nav2.md`](workload-nav2.md) — bringing up the ROS 2 / Nav2 workload the availability test runs against
- [`gz-split/`](gz-split/) — adding Gazebo and SLAM to that workload, with the simulator on the x86 host
- [`yolo/`](yolo/) — the AI inference co-tenant, on a self-built riscv64 ONNX Runtime
- [`ras-ros/`](ras-ros/) — publishing the errors injected here onto ROS 2, and into the event log the TSP names
- [q-vela `docs/ras.md`](https://github.com/riscv-vela/q-vela/blob/main/docs/ras.md) — boot command and the RERI smoke test
- [vela-fws](https://github.com/riscv-vela/vela-fws) — firmware and root filesystem
- [vela-linux](https://github.com/riscv-vela/vela-linux) — kernel, including the RAS options
- [`../reri register generator/`](../reri%20register%20generator/) — spreadsheet for working out register bit fields
- [RERI specification](https://github.com/riscv-non-isa/riscv-ras-eri)
