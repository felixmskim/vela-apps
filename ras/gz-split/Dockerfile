# Gazebo Harmonic + Nav2 + slam_toolbox on the x86 host, headless.
#
# The host has no usable GPU (Matrox BMC VGA, no /dev/dri/renderD* node), so all
# rendering is Mesa llvmpipe. The lidar is small enough for that. For a
# 360-sample scan over 360 degrees, Ogre2GpuRays::ConfigureCamera() computes
# (pi/2 / hFOV) * samples = 90, rounds up to a power of two, and clamps to a
# minimum of 128 (Ogre2GpuRays.cc: min1stPassSamples = 128u), so each cube face
# is 128x128; only the faces the sample directions actually hit get rendered --
# four, for a planar scan. At 5 Hz that is ~0.33 Mpix/s of first-pass raster.
# That figure is derived from the source, not profiled, and it is a lower bound:
# each face renders a depth target alongside the colour one. The number that
# decides anything is the measured real-time factor from run_rtf.sh (G1).
#
# ros:jazzy-ros-base, not -desktop: rviz is never used here and would add ~1 GB.
#
# Pinned by digest deliberately. `ros:jazzy-ros-base` is a moving tag, and this
# image has to be rebuildable months from now to re-check a number in RUNLOG.md.
# To refresh the pin:  docker buildx imagetools inspect ros:jazzy-ros-base
#                      (take the top-level index Digest)
# To float it for one build without editing this file:
#                      docker build --build-arg BASE_IMAGE=ros:jazzy-ros-base .
ARG BASE_IMAGE=ros:jazzy-ros-base@sha256:da725acf8b0f9f30c683e33ffbdcd6482d077af96d6fdc7688c5f4f280b7d923
FROM ${BASE_IMAGE}

# The apt versions below are deliberately *not* pinned with `=`. packages.ros.org
# keeps only the newest build of each package, so a pin like
# `ros-jazzy-rmw-fastrtps-cpp=8.4.4-1noble.20260615.124621` stops resolving days
# after it is written: it makes the build fail rather than reproduce. What gets
# recorded instead is what this build actually resolved to --
# /opt/vela/apt-manifest.txt, copied out in step 1 of the run log.
#
# Why each non-ROS package is here:
#   libgl1-mesa-dri, libegl-mesa0, libglx-mesa0  llvmpipe plus the EGL and GLX
#         loaders that gz-rendering's ogre2 backend opens.
#   mesa-utils  eglinfo and glxinfo, printed by run_rtf.sh step 1 as the
#         render-path evidence. On noble it is a metapackage that pulls
#         mesa-utils-bin, which carries both binaries (verified in the built
#         image). The mesa-utils-extra that used to hold eglinfo now installs no
#         binaries at all, so it was dropped after checking its file list.
#   xvfb  fallback rung: ogre2 without --headless-rendering, under xvfb-run.
#   python3-numpy  score_map.py -- PGM parsing and the occupied-cell IoU.
# iproute2 was dropped for the same reason: the container runs with
# --network host, so the host's own `ss` already sees these sockets.
#
# ros-jazzy-nav2-minimal-tb4-sim is what the TSP figure's robot needs. It pulls
# nav2-minimal-tb4-description (the URDF and meshes) and robot_state_publisher
# as hard dependencies, so neither is listed separately.
#
# ros-jazzy-rmw-fastrtps-cpp looks redundant -- ros-base already installs it --
# but only as an alternative: ros-jazzy-rmw-implementation declares
# "rmw-fastrtps-cpp | rmw-cyclonedds-cpp | rmw-connextdds", and apt merely
# happens to pick the first. Both DDS profiles here are Fast DDS-specific, so the
# assumption is named rather than inherited.
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-nav2-minimal-tb3-sim \
      ros-jazzy-nav2-minimal-tb4-sim \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-xacro \
      ros-jazzy-rmw-fastrtps-cpp \
      libgl1-mesa-dri libegl-mesa0 libglx-mesa0 mesa-utils \
      xvfb python3-numpy \
 && rm -rf /var/lib/apt/lists/*

# Software rendering, stated rather than assumed. Neither variable is
# load-bearing on this host -- there is no /dev/dri/renderD* for Mesa to open, so
# EGL device enumeration returns EGL_MESA_device_software and nothing else -- and
# LIBGL_ALWAYS_SOFTWARE acts on the GLX path, with an EGL reach that varies by
# Mesa version. They are set so the choice is explicit, and GALLIUM_DRIVER pins
# the software driver to llvmpipe rather than leaving softpipe possible.
#
# Nothing here is evidence. run_rtf.sh step 1 prints eglinfo and glxinfo, and
# step 3 greps the engine's own initialisation lines out of the gz log; those two
# outputs go in RUNLOG.md and are what settle whether the path is what we think.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe

# First entry: model://turtlebot3_world, which needs the models directory itself.
# Second: package://nav2_minimal_tb4_description/meshes/... -- for a scheme other
# than model://, gz-common's SystemPaths::FindFileURI joins authority + path onto
# each search path, so `share` is exactly the right level. This is also what
# upstream nav2_minimal_tb4_sim/launch/simulation.launch.py sets.
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# The reproducibility record travels inside the image: which base it came from,
# and every package version apt resolved. Copy both out after building:
#   docker exec vela-gz cat /opt/vela/apt-manifest.txt > logs/s1-manifest.txt
ARG BASE_IMAGE
LABEL org.opencontainers.image.base.name="${BASE_IMAGE}"
RUN mkdir -p /opt/vela \
 && echo "${BASE_IMAGE}" > /opt/vela/base-image.txt \
 && dpkg-query -W -f='${Package}\t${Version}\n' | sort > /opt/vela/apt-manifest.txt

# Run as the invoking user, not root. Everything this container produces lands in
# the bind-mounted /work, which is a git working tree; root-owned run artefacts
# there need sudo to clean up. ubuntu:noble already ships a uid-1000 user, so
# delete whoever holds the uid rather than special-case it -- the account is
# always `vela` with a real home, which matters because gz caches Fuel models
# under $HOME/.gz and slam_toolbox writes $HOME/.ros.
# For a host account that is not uid 1000:
#   docker build --build-arg HOST_UID=$(id -u) --build-arg HOST_GID=$(id -g) .
# Named HOST_UID, not UID: bash treats $UID as readonly, so an ARG called UID
# would silently resolve to 0 if the build shell were ever switched to bash.
ARG HOST_UID=1000
ARG HOST_GID=1000
RUN userdel -r "$(getent passwd ${HOST_UID} | cut -d: -f1)" 2>/dev/null || true; \
    groupadd -g ${HOST_GID} vela 2>/dev/null || true; \
    useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash vela \
 && mkdir -p /work && chown ${HOST_UID}:${HOST_GID} /work
USER vela
WORKDIR /work

# The run command belongs in the image, so `docker run -d --name vela-gz ...
# vela-gzsim:jazzy` needs no trailing argument. Note that the scripts drive the
# container with `docker exec`, which does not run the entrypoint -- that is why
# every dex() helper sources /opt/ros/jazzy/setup.bash itself. To install a
# package into a running container, use `docker exec -u 0`.
CMD ["sleep", "infinity"]
