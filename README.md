# OpenMower Humble Wrapper

This directory is a new, separate ROS 2 Humble-oriented wrapper around [`NFAZ10/openmowernext`](https://github.com/NFAZ10/openmowernext), which is a ROS 2 Jazzy port of OpenMower.

It leaves the original ROS 1 wrapper in `/home/nfazio/openmower-docker` untouched.

## What this does

- Builds a local `ros:humble` image that clones a pinned `openmowernext` commit.
- Applies a small compatibility patch set for Humble and external runtime config.
- Runs the ROS 2 stack from Docker in its own `openmower-humble/` directory.
- Keeps Mosquitto and the optional OpenMower UI in this new directory as separate services.
- Defaults to an external-hardware mode that is meant to coexist with a separate `rosmower` container on the same ROS 2 host network.
- Starts a small compatibility bridge that publishes the legacy OpenMower UI MQTT/BSON topics and the joystick WebSocket on port `9002`.

## What this does not promise

- This is a best-effort Humble adaptation, not an official upstream Humble release.
- Gazebo simulation packages are intentionally skipped in the Humble build.
- The micro-ROS agent is intentionally omitted from this wrapper for now.
- Hardware behavior, docking, and UI compatibility still need real-world validation.
- The default runtime assumes another container owns the physical devices and that this stack should consume shared topics instead of opening serial hardware directly.
- The compatibility bridge now publishes a supported legacy `mower_logic:*` subset for:
  - starting mowing by generating a simple in-area mowing path and sending it to Nav2 while still toggling the shared `/robot_mode_cmd` topic
  - pausing/resuming/stopping mowing by canceling/resuming the bridge-generated Nav2 route while still coordinating with the shared `/robot_mode_cmd` topic
  - entering/exiting area recording mode
  - starting/resuming area recording
  - discarding a recording
  - auto/manual point collection toggles
  - manual boundary point insertion
  - docking station recording
  - manual mowing enable/disable while area recording is active
  - `skip_area` is still not mapped because the shared ROSMower mode manager only exposes coarse runtime modes (`idle`, `charging`, `mowing`, `full`) instead of the full legacy mowing FSM.
- Area recording still depends on the Humble stack providing a valid `map` transform at runtime; without that TF frame, the upstream `RecordAreaBoundary` action aborts before recording can continue.
- The default Humble config leaves `ekf_se_map` disconnected from `/mavros/global_position/local` because some external-hardware MAVROS setups publish invalid local-position values there; if you want to re-enable that input later, verify the topic contains finite coordinates first.

## Files

- `Dockerfile` builds the Humble image.
- `scripts/apply-humble-adaptation.py` rewrites the pinned upstream files for Humble and external runtime config.
- `openmower-config/humble/` contains mounted runtime config files.
- `openmower-config/reference/` keeps the old local placeholder configs for reference.
- `scripts/fetch-upstream.sh` optionally checks out the pinned upstream source locally.

## Build and run

```bash
cd /home/nfazio/openmower-docker/openmower-humble
cp .env.example .env
docker compose up --build
```

To start the optional UI:

```bash
docker compose -f docker-compose.ui.yml up --build -d
```

## Helper script

For a more ergonomic workflow similar to the rosmower helper, use:

```bash
./docker-helper.sh help
./docker-helper.sh init-env
./docker-helper.sh up-build -d
./docker-helper.sh ui -d
./docker-helper.sh logs core
./docker-helper.sh shell main
```

The helper wraps the core stack, optional UI, status/log commands, and common container shell/exec tasks while keeping the compose project rooted in this directory.

## Notes

- The main service uses `network_mode: host` for ROS 2 discovery.
- Mosquitto defaults to host port `1884` for raw MQTT and `9001` for MQTT-over-WebSocket so it does not collide with the original wrapper's `1883`.
- The UI compatibility bridge listens on host port `9002` for the OpenMower web app's joystick socket.
- By default, the UI bridge also listens to `/robot_mode` and publishes `/robot_mode_cmd` so the legacy Start/Stop buttons can control the shared ROSMower mode manager on the same ROS domain.
- The legacy `Start` button now uses the first suitable mowing area from `/mowing_map` to build a simple boustrophedon route and send it through `/navigate_through_poses`, which in turn produces `cmd_vel` for the external ROSMower stack.
- The bridge-side mowing planner is intentionally simple; tune `OM_UI_BRIDGE_MOW_STRIPE_SPACING`, `OM_UI_BRIDGE_MOW_BOUNDARY_MARGIN`, and `OM_UI_BRIDGE_MOW_WAYPOINT_TOLERANCE` in `.env` to better match your deck width and localization accuracy.
- The launch files are patched to read config from `/config`, which is mounted from `./openmower-config/humble`.
- The pinned upstream commit is recorded in `upstream/COMMIT.txt`.
- The default `.env` now matches `rosmower_launch` with `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- In external-hardware mode, this stack publishes motion commands to `/cmd_vel` and expects shared `/odom`, `/mavros/imu/data`, and `/mavros/global_position/local` topics from the other container.
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` is enabled by default so Fast DDS uses UDP between containers instead of trying shared-memory transport across private Docker IPC namespaces.
