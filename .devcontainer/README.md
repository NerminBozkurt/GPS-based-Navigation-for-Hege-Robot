# Devcontainer — UNVERIFIED

**This container has never built successfully.** The first attempt failed and
the error was not diagnosed, because the PX4 work moved to a machine that
already had Gazebo Harmonic installed. Treat everything here as a draft.

If you are looking for a working Harmonic + PX4 environment, the one in
[oguzissik/hege_gps_navigation](https://github.com/oguzissik/hege_gps_navigation)
is known to work — it is what this was adapted from.

## Why it exists

Gazebo Classic and Gazebo Harmonic cannot be installed on the same machine:
their Debian packages both ship `/usr/bin/gz` and conflict outright. PX4 v1.16
SITL requires Harmonic, and this repository's default simulation runs on
Classic. A container is one way to have both without either apt-removing the
other. See `docs/gazebo_harmonic.md`.

## What is here

| File | What it does |
| --- | --- |
| `Dockerfile` | ROS 2 Humble, Gazebo Harmonic, PX4's build toolchain, the Micro XRCE-DDS Agent, `ros_gz` for Harmonic |
| `devcontainer.json` | Mounts, X11, host networking, GPU flags (commented out by default) |
| `post-create.sh` | Clones `px4_msgs`, builds `gz_ros2_control` from source, builds the workspace |
| `setup-px4.sh` | Clones PX4 v1.16.1 and installs the Hege model and airframe — separate because it is ~2 GB |

Three things in the Dockerfile are additions to the upstream one and are the
parts worth keeping whoever finishes this:

- the **Micro XRCE-DDS Agent**, built from source. Without it no `/fmu/out/*`
  topic ever appears and both `hege_px4_bridge` and `hege_px4_sensors` sit
  reading nothing;
- **`ros_gz` from the OSRF repository** under its `gzharmonic` name, because
  the `ros-humble-ros-gz-*` binaries are built against Gazebo Fortress;
- a **source build of `gz_ros2_control`** for the same reason —
  `ros-humble-gz-ros2-control` depends on `libsdformat12`, which is Fortress,
  where Harmonic is `libsdformat14`.

## To pick this up

Build it directly rather than through VS Code, which hides the error:

```bash
docker build -f .devcontainer/Dockerfile -t hege-test .
```

The failure is most likely one of: a package name in the OSRF repository, the
XRCE agent build needing a dependency that is not installed, or a download
timing out. None of those was confirmed.
