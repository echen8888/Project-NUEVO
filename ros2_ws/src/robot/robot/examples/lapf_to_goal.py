"""
lapf_to_goal.py — leashed APF goal seeking with lidar
=====================================================
Single-goal obstacle avoidance using the new `lapf_to_goal()` API.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/lapf_to_goal.py main.py
    ros2 run robot robot

Press BTN_1 to start the run. BTN_2 cancels the active motion.

WHAT THIS TEACHES
-----------------
1. Optional lidar and GPS setup for local planning
2. Non-blocking `lapf_to_goal()` inside the standard FSM loop
3. How to inspect the moving virtual target while the rover runs
4. How the current default LAPF tuning behaves on one goal
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    Motor,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
)
from robot.robot import FirmwareState, Robot, Unit


# Shared lidar/GPS hardware calibration lives in robot/hardware_map.py.
# If you need to change lidar mount, lidar self-filtering, or GPS tag body
# offset values, edit ros2_ws/src/robot/robot/hardware_map.py.
ENABLE_LIDAR = True
ENABLE_GPS = False

# IMPORTANT: update TAG_ID to match your robot when GPS is enabled.
TAG_ID = -1


POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

GOAL_MM = (0.0, 2000.0)
VELOCITY_MM_S = 150.0
TOLERANCE_MM = 50.0
MAX_ANGULAR_RAD_S = 1.0

# Default LAPF tuning now lives on Robot. Keep these as `None` to use the
# current runtime defaults, or set them explicitly here while tuning.
LEASH_LENGTH_MM = None
REPULSION_RANGE_MM = None
TARGET_SPEED_MM_S = None
REPULSION_GAIN = None
ATTRACTION_GAIN = None
FORCE_EMA_ALPHA = None
INFLATION_MARGIN_MM = None
LEASH_HALF_ANGLE_DEG = None

STATUS_PRINT_INTERVAL_S = 0.5


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


def print_status(robot: Robot) -> None:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        label = "fused"
    else:
        x, y, theta = robot.get_odometry_pose()
        label = "odom "

    virtual_target = robot.get_virtual_target()
    obstacle_tracks = robot.get_obstacle_tracks()
    if virtual_target is None:
        vt_summary = " vt=(none)"
    else:
        vt_summary = f" vt=({virtual_target[0]:6.0f}, {virtual_target[1]:6.0f}) mm"

    print(
        f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°"
        f"{vt_summary} tracked={len(obstacle_tracks)}"
    )


def start_goal(robot: Robot):
    return robot.lapf_to_goal(
        GOAL_MM[0],
        GOAL_MM[1],
        velocity=VELOCITY_MM_S,
        tolerance=TOLERANCE_MM,
        leash_length=LEASH_LENGTH_MM,
        repulsion_range=REPULSION_RANGE_MM,
        target_speed=TARGET_SPEED_MM_S,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_gain=REPULSION_GAIN,
        attraction_gain=ATTRACTION_GAIN,
        force_ema_alpha=FORCE_EMA_ALPHA,
        inflation_margin_mm=INFLATION_MARGIN_MM,
        leash_half_angle_deg=LEASH_HALF_ANGLE_DEG,
        blocking=False,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start LAPF goal, BTN_2 to cancel")
            print(f"[CFG] goal={GOAL_MM} velocity={VELOCITY_MM_S:.0f} mm/s tolerance={TOLERANCE_MM:.0f} mm")
            print(
                f"[CFG] LAPF defaults: leash={robot.LAPF_LEASH_LENGTH_MM:.0f} mm "
                f"half_angle={robot.LAPF_LEASH_HALF_ANGLE_DEG:.0f}° "
                f"target_speed={robot.LAPF_TARGET_SPEED_MM_S:.0f} mm/s "
                f"repulsion_range={robot.LAPF_REPULSION_RANGE_MM:.0f} mm "
                f"inflation={robot.LAPF_INFLATION_MARGIN_MM:.0f} mm"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
                    f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
                )
            if ENABLE_GPS:
                print(
                    f"[CFG] gps tag_id={TAG_ID} tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, "
                    f"{TAG_BODY_OFFSET_Y_MM:.0f}) mm"
                )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                motion_handle = start_goal(robot)
                last_status_print_at = now
                print("[FSM] MOVING — LAPF goal started")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — LAPF goal cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] DONE — goal complete")
                    print_status(robot)
                    motion_handle = None
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — press BTN_1 to run again")
                    state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
