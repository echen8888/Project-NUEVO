"""
TLV Type Definitions - Auto-generated from TLV_TypeDefs.json
This file is auto-generated - DO NOT EDIT MANUALLY
"""

# ============================================================================
# TLV Type Constants
# ============================================================================

SYS_HEARTBEAT = 1
SYS_STATUS = 2
SYS_CMD = 3
SYS_CONFIG = 4
SYS_SET_PID = 5
DC_ENABLE = 16
DC_SET_POSITION = 17
DC_SET_VELOCITY = 18
DC_SET_PWM = 19
DC_STATUS_ALL = 20
STEP_ENABLE = 32
STEP_SET_PARAMS = 33
STEP_MOVE = 34
STEP_HOME = 35
STEP_STATUS_ALL = 36
SERVO_ENABLE = 48
SERVO_SET = 49
SERVO_STATUS_ALL = 50
SENSOR_IMU = 64
SENSOR_KINEMATICS = 65
SENSOR_VOLTAGE = 66
SENSOR_RANGE = 67
SENSOR_MAG_CAL_CMD = 68
SENSOR_MAG_CAL_STATUS = 69
IO_SET_LED = 80
IO_SET_NEOPIXEL = 81
IO_STATUS = 82

# Dictionary for programmatic access
TLV_TYPES = {
    'SYS_HEARTBEAT': 1,
    'SYS_STATUS': 2,
    'SYS_CMD': 3,
    'SYS_CONFIG': 4,
    'SYS_SET_PID': 5,
    'DC_ENABLE': 16,
    'DC_SET_POSITION': 17,
    'DC_SET_VELOCITY': 18,
    'DC_SET_PWM': 19,
    'DC_STATUS_ALL': 20,
    'STEP_ENABLE': 32,
    'STEP_SET_PARAMS': 33,
    'STEP_MOVE': 34,
    'STEP_HOME': 35,
    'STEP_STATUS_ALL': 36,
    'SERVO_ENABLE': 48,
    'SERVO_SET': 49,
    'SERVO_STATUS_ALL': 50,
    'SENSOR_IMU': 64,
    'SENSOR_KINEMATICS': 65,
    'SENSOR_VOLTAGE': 66,
    'SENSOR_RANGE': 67,
    'SENSOR_MAG_CAL_CMD': 68,
    'SENSOR_MAG_CAL_STATUS': 69,
    'IO_SET_LED': 80,
    'IO_SET_NEOPIXEL': 81,
    'IO_STATUS': 82,
}

# Reverse map: integer type id → name string
TLV_NAMES = {v: k for k, v in TLV_TYPES.items()}
