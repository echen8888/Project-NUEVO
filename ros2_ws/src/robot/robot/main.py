"""
main.py — student entry point
==============================
This is the only file you need to edit.

`run(robot)` is called by robot_node.py after ROS is set up and the Robot
instance is ready. Write your FSM here and call fsm.spin() at the end.

To run:
    ros2 run robot robot
"""

from robot.robot import Robot, FirmwareState
from robot.robot_fsm import RobotFSM
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.path_planner import PurePursuitPlanner
from robot.path_planner import PurePursuitPlanner2
import numpy as np


# Drive-wheel mapping for this robot build.
# Change these if the left/right wheels are wired to different DC motor ports.
LEFT_WHEEL_MOTOR = Motor.DC_M1
RIGHT_WHEEL_MOTOR = Motor.DC_M2


class MyFSM(RobotFSM):
    """
    Three-state FSM: INIT → IDLE ↔ MOVING

    INIT  : one-time startup — starts the firmware, then immediately goes to IDLE
    IDLE  : waiting; RED LED on. Press button 1 to start moving.
    MOVING: driving forward; GREEN LED on. Press button 2 to stop.

    ┌──────┐  ready      ┌────────┐  to_moving  ┌────────┐
    │ INIT │ ──────────> │  IDLE  │ ──────────> │ MOVING │
    └──────┘             │        │ <────────── │        │
                         └────────┘   to_idle   └────────┘
    """

    # ------------------------------------------------------------------
    # Part 1 — State machine setup
    #
    # add_transition(from_state, event, to_state, action)
    #   from_state : which state this transition starts from
    #   event      : the name you pass to trigger() to fire it
    #   to_state   : which state to move to
    #   action     : method to call once, at the moment of transition
    #
    # The event name and action method share the same word so it is easy
    # to trace:  trigger("to_moving") → _on_to_moving()
    # ------------------------------------------------------------------

    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="INIT")

        self.add_transition("INIT",   "ready",     "IDLE",   action=self._on_ready)
        self.add_transition("IDLE",   "to_moving", "MOVING", action=self._on_to_moving)
        self.add_transition("MOVING", "to_idle",   "IDLE",   action=self._on_to_idle)

    # ------------------------------------------------------------------
    # Part 2 — Continuous logic (runs every spin cycle, ~50 Hz by default)
    #
    # update() is called repeatedly by spin(). Use it to:
    #   - read sensors or buttons
    #   - send continuous commands (LEDs, velocity adjustments)
    #   - decide when to trigger a transition
    #
    # Calling self.robot.xxx() here talks directly to hardware but does
    # NOT change the FSM state — it just sends a command right now.
    # Calling self.trigger("event") is what actually changes state.
    # ------------------------------------------------------------------

    def update(self) -> None:
        state = self.get_state()

        if state == "INIT":
            # No output — transition happens automatically on the first cycle.
            # trigger() changes state to IDLE and calls _on_ready() once.
            self.path = np.array([
            [0.0, 0.0],
            [0.0, 500.0],
            [500.0, 500.0],
            [500.0, 0.0]
            ])
            #self.planner = PurePursuitPlanner2(lookahead_distance=50.0, max_linear_speed=150.0, goal_tolerance=20.0) # mm, mm/s
            self.planner = PurePursuitPlanner(lookahead_dist=50.0, max_angular=2.0, goal_tolerance=20.0) # mm, rad/s
            self.trigger("ready")

        elif state == "IDLE":
            # This demo only listens to one button in each state, so polling
            # the current level is enough. Keep LED writes out of update() so
            # button handling stays responsive and the bridge does not get
            # flooded with repeated output commands.
            CurrentX, CurrentY, CurrentTheta = self.robot.get_pose() # Get current pose from the robot (x, y in mm; theta in radians)
            print(f"IDLE Pose is ({CurrentX:.2f}, {CurrentY:.2f}, {CurrentTheta:.2f} rad).  Press button 1 to start moving.")
        
            if self.robot.get_button(Button.BTN_1):
                self.trigger("to_moving")

        elif state == "MOVING":

            CurrentX, CurrentY, CurrentTheta = self.robot.get_pose() # Get current pose from the robot (x, y in mm; theta in radians)
            print(f"MOVING: Current pose is ({CurrentX:.2f}, {CurrentY:.2f}, {CurrentTheta:.2f} rad).")
            
            #CurrentPursuitX, CurrentPursuitY = self.planner._lookahead_point(self.path, CurrentX, CurrentY) # Get current lookahead point for debugging
            CurrentPursuitX, CurrentPursuitY = self.planner._lookahead_point(CurrentX, CurrentY, self.path) # Get current lookahead point for debugging
            print(f"MOVING: Current lookahead point is ({CurrentPursuitX:.2f}, {CurrentPursuitY:.2f}).")
            #LinearVelocityCmd, AngularVelocityCmd = self.planner.compute_velocity( # Get velocity command from the planner (linear in mm/s, angular in rad/s)
            #    path=self.path,
            #    pose=(CurrentX, CurrentY, CurrentTheta)
            #)
            LinearVelocityCmd, AngularVelocityCmd = self.planner.compute_velocity( # Get velocity command from the planner (linear in mm/s, angular in rad/s)
                pose=(CurrentX, CurrentY, CurrentTheta),
                waypoints=self.path,
                max_linear = 150.0, # mm/s
            )
            print(f"MOVING: Computed command: {LinearVelocityCmd:.2f} mm/s, {AngularVelocityCmd:.2f} rad/s.")
            self.robot.set_velocity(LinearVelocityCmd, AngularVelocityCmd) # Send velocity command to the robot. This does NOT change the FSM state.
            
            if self.planner.TargetReached(CurrentX, CurrentY, self.path): # Check if the target is reached. If so, trigger transition to IDLE.
                print("MOVING: Target reached! Stopping.")
                self.trigger("to_idle")

            #if self.robot.get_button(Button.BTN_2):
            #    self.trigger("to_idle")

    # ------------------------------------------------------------------
    # Part 3 — Transition actions (called once, at the moment of change)
    #
    # These run exactly once when the transition fires — not every cycle.
    # Use them for one-shot commands: enabling the firmware, setting an
    # initial velocity, or cleanly stopping motors.
    # ------------------------------------------------------------------

    def _on_ready(self) -> None:
        """Called once when leaving INIT. Starts the firmware."""
        self.robot.set_state(FirmwareState.RUNNING)
        self._show_idle_leds()
        print("[FSM] IDLE")

    def _on_to_moving(self) -> None:
        """Called once when entering MOVING."""
        self._show_moving_leds()
        self.robot.set_velocity(100, 0.0)  # 100 mm/s forward, 0 deg/s rotation
        print("[FSM] MOVING")

    def _on_to_idle(self) -> None:
        """Called once when entering IDLE from MOVING."""
        self.robot.stop()
        self._show_idle_leds()
        print("[FSM] IDLE")

    def _show_idle_leds(self) -> None:
        self.robot.set_led(LED.GREEN, 0)
        self.robot.set_led(LED.RED, 255)

    def _show_moving_leds(self) -> None:
        self.robot.set_led(LED.RED, 0)
        self.robot.set_led(LED.GREEN, 255)


def run(robot: Robot) -> None:
    robot.set_left_wheel(LEFT_WHEEL_MOTOR)
    robot.set_right_wheel(RIGHT_WHEEL_MOTOR)
    fsm = MyFSM(robot)
    fsm.spin(hz=DEFAULT_FSM_HZ)
