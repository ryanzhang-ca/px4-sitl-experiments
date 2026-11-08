# PX4 SITL Experiments

Hands-on experiments with the PX4 autopilot in software-in-the-loop (SITL) simulation: flying the vehicle, testing failsafe behaviour under connectivity and sensor loss, and controlling it over MAVLink with a Python script.

I'm building a physical quadcopter (F450 frame, Pixhawk 6C, Raspberry Pi companion computer). I'm using simulation to test behaviour safely before running it on real hardware.

## Setup

| Component | Version / details |
|---|---|
| OS | Windows 11 with WSL2, Ubuntu 22.04 |
| Autopilot | PX4-Autopilot (main branch), built from source |
| Simulator | Gazebo, `gz_x500` quadcopter model |
| Ground control | QGroundControl 4.4.3 (Linux AppImage, running in WSL) |
| Scripting | Python 3.10, pymavlink |

```bash
# Build and launch the simulator
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# In a second terminal: ground control station
./QGroundControl.AppImage

# In a third terminal: MAVLink script
pip install pymavlink
python3 px4_takeoff_land.py
```

## Failsafe tests

Each test was run in SITL from the `pxh>` shell and observed in the PX4 console and QGroundControl.

### Test A: Loss of ground control station link

| | |
|---|---|
| **Setup** | `param set NAV_DLL_ACT 2` (data link loss action = Return) |
| **Action** | `commander takeoff`, then closed QGroundControl while hovering |
| **Expected** | Vehicle detects the lost link and returns to launch |
| **Result** | Link loss detected → **Hold for 5 s** (grace period, `COM_FAIL_ACT_T`) → **Return** → landed at home → disarmed |

```
INFO  [commander] Connection to ground station lost
WARN  [failsafe] Failsafe activated: entering Hold for 5 seconds
WARN  [failsafe] Failsafe activated
INFO  [navigator] Return: start return at 491 m (3 m above destination)
INFO  [navigator] Return: land at destination
INFO  [commander] Landing detected
INFO  [commander] Disarmed by landing
```

**Takeaway:** PX4 handles link loss in stages: detect, give the operator a short grace period, then run the configured action. The vehicle still had a valid position estimate, so it could fly home.

<!-- Add screenshot: ![Test A](images/test_a.png) -->

### Test B: GPS failure in flight

| | |
|---|---|
| **Setup** | `param set SYS_FAILURE_EN 1` (enable failure injection) |
| **Action** | `commander takeoff`, then `failure gps off` while hovering; later `failure gps ok` |
| **Expected** | Vehicle falls back to a safe behaviour after losing position |
| **Result** | Position setpoints invalid → **blind land** (descend in place) → GPS restored mid-descent, landing continued → disarmed |

```
WARN  [failure_injection_manager] Injected: gps off, all instances
WARN  [mc_pos_control] invalid setpoints
WARN  [mc_pos_control] Failsafe: blind land
WARN  [failsafe] Failsafe activated
WARN  [failure_injection_manager] Cleared: gps, all instances
INFO  [commander] Landing detected
INFO  [commander] Disarmed by landing
```

**Takeaway:** Without GPS, PX4 has no horizontal position estimate, so it can't hold position or return home. Its only safe option is to descend in place. Once committed to the landing failsafe, it finished landing even after GPS came back.

<!-- Add screenshot: ![Test B](images/test_b.png) -->

### Comparison

| | Test A: link loss | Test B: GPS loss |
|---|---|---|
| Position estimate | Still valid | Lost |
| Failsafe action | Hold → Return | Blind land |
| Outcome | Landed at home | Landed in place |

PX4 picks its failsafe based on what information it still has. Any system that can lose or stop trusting a data source, whether the control link or a sensor, needs a planned fallback for each case.

## MAVLink control script: `px4_takeoff_land.py`

A Python script using pymavlink that connects to PX4 over UDP (port 14540) and flies a simple mission:

1. Wait for a `HEARTBEAT` to confirm the link
2. Arm (`MAV_CMD_COMPONENT_ARM_DISARM`), retrying until PX4's preflight checks pass
3. Take off (`MAV_CMD_NAV_TAKEOFF`) to the default altitude (`MIS_TAKEOFF_ALT`, 2.5 m)
4. Print altitude from `GLOBAL_POSITION_INT`
5. Land (`MAV_CMD_NAV_LAND`)

Each command is sent as a `COMMAND_LONG` and the script waits for the matching `COMMAND_ACK`.

### Bugs found and fixed

**1. Takeoff sent without confirming the arm succeeded**

The first version sent the arm command, then immediately sent takeoff. PX4 replied `MAV_RESULT_TEMPORARILY_REJECTED` to the arm because preflight checks hadn't passed yet, but accepted the takeoff command. The vehicle switched into Takeoff mode while disarmed, so nothing happened. Altitude stayed at 0 m.

*Fix:* check the arm ACK, retry up to 20 times, print PX4's `STATUSTEXT` message explaining each rejection, and abort if arming never succeeds.

**2. Stale telemetry**

The altitude printout showed 0 m during takeoff and about 2.3 m during landing, which is backwards. The script read one `GLOBAL_POSITION_INT` message and then slept for 0.5 s, but PX4 streams that message many times per second. Unread messages queued in the buffer, so each printout showed older and older data.

*Fix:* read every incoming message without sleeping, and only print twice per second, so the printed value is always the newest.

### Result after fixes

```
Heartbeat from system 1, component 0
Arming...
  ACK for command 400: MAV_RESULT_ACCEPTED
Taking off...
  ACK for command 22: MAV_RESULT_ACCEPTED
  relative alt: 0.00 m → 0.35 m → 1.13 m → 1.76 m → 2.12 m → 2.28 m
Landing...
  ACK for command 21: MAV_RESULT_ACCEPTED
  relative alt: 2.31 m → 1.86 m → 1.22 m → 0.50 m → 0.00 m
```

Altitude now tracks the vehicle in real time: a smooth climb toward 2.5 m, then a smooth descent to the ground.

<!-- Add screenshot: ![Script output](images/script_output.png) -->

## Next steps

- **MAVLink 2 message signing:** enable signing in SITL, then confirm that unsigned or wrongly-signed commands are rejected and that replayed messages fail the timestamp check
- **Authentication failure handling:** define how the vehicle should respond to repeated invalid commands, using the same staged approach as PX4's existing failsafes
- **Hardware:** connect the Raspberry Pi companion computer to the Pixhawk over UART (TELEM2) and run the same script on the real vehicle
