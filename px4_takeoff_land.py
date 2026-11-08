"""
Minimal pymavlink script for PX4 SITL:
connect -> wait for heartbeat -> arm -> take off -> hover -> land.

Run PX4 SITL first (make px4_sitl gz_x500), then:
    pip install pymavlink
    python3 px4_takeoff_land.py
"""
import math
import time

from pymavlink import mavutil

# PX4 SITL sends MAVLink for offboard/API use on UDP 14540 (QGC uses 14550).
CONNECTION = "udpin:0.0.0.0:14540"


def send_command(master, command, params):
    """Send a COMMAND_LONG and wait for its COMMAND_ACK."""
    p = list(params) + [0] * (7 - len(params))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,  # confirmation
        *p,
    )
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if ack is None:
        print(f"  no ACK for command {command}")
    else:
        result = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
        print(f"  ACK for command {ack.command}: {result}")
    return ack


def print_altitude(master, seconds):
    """Print relative altitude for a few seconds.

    Keep reading every incoming message (no sleeping) so we always see the
    newest data, but only print twice a second. Sleeping between reads lets
    old messages pile up in the buffer, so the printout falls behind reality.
    """
    end = time.time() + seconds
    last_print = 0.0
    while time.time() < end:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg and time.time() - last_print >= 0.5:
            print(f"  relative alt: {msg.relative_alt / 1000:.2f} m")
            last_print = time.time()


def main():
    master = mavutil.mavlink_connection(CONNECTION)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system {master.target_system}, "
          f"component {master.target_component}")

    # PX4 rejects arming until its preflight checks pass, so retry
    # and print PX4's own STATUSTEXT messages explaining why.
    print("Arming...")
    armed = False
    for attempt in range(1, 21):
        ack = send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1])
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            armed = True
            break
        status = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if status:
            print(f"  PX4 says: {status.text}")
        print(f"  arming attempt {attempt} failed, retrying...")
        time.sleep(1)

    if not armed:
        print("Could not arm. Run 'commander check' at the pxh> prompt to see why.")
        return

    # NaN for position/altitude tells PX4 to use its defaults
    # (current position, takeoff altitude from MIS_TAKEOFF_ALT).
    print("Taking off...")
    nan = math.nan
    send_command(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                 [nan, 0, 0, nan, nan, nan, nan])
    print_altitude(master, 10)

    print("Landing...")
    send_command(master, mavutil.mavlink.MAV_CMD_NAV_LAND,
                 [0, 0, 0, nan, nan, nan, nan])
    print_altitude(master, 15)

    print("Done.")


if __name__ == "__main__":
    main()
