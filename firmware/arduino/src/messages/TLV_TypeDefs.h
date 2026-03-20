#pragma once

#include <stdint.h>

// Define TLV type constants here so both server and client can use them:
// TLV type identifiers are 8-bit values on the wire.
// This file is auto-generated from TLV_TypeDefs.json - DO NOT EDIT MANUALLY

// ============================================================================
// TLV Type Constants
// ============================================================================

constexpr uint8_t SYS_HEARTBEAT = 1U;
constexpr uint8_t SYS_STATUS = 2U;
constexpr uint8_t SYS_CMD = 3U;
constexpr uint8_t SYS_CONFIG = 4U;
constexpr uint8_t SYS_SET_PID = 5U;
constexpr uint8_t DC_ENABLE = 16U;
constexpr uint8_t DC_SET_POSITION = 17U;
constexpr uint8_t DC_SET_VELOCITY = 18U;
constexpr uint8_t DC_SET_PWM = 19U;
constexpr uint8_t DC_STATUS_ALL = 20U;
constexpr uint8_t STEP_ENABLE = 32U;
constexpr uint8_t STEP_SET_PARAMS = 33U;
constexpr uint8_t STEP_MOVE = 34U;
constexpr uint8_t STEP_HOME = 35U;
constexpr uint8_t STEP_STATUS_ALL = 36U;
constexpr uint8_t SERVO_ENABLE = 48U;
constexpr uint8_t SERVO_SET = 49U;
constexpr uint8_t SERVO_STATUS_ALL = 50U;
constexpr uint8_t SENSOR_IMU = 64U;
constexpr uint8_t SENSOR_KINEMATICS = 65U;
constexpr uint8_t SENSOR_VOLTAGE = 66U;
constexpr uint8_t SENSOR_RANGE = 67U;
constexpr uint8_t SENSOR_MAG_CAL_CMD = 68U;
constexpr uint8_t SENSOR_MAG_CAL_STATUS = 69U;
constexpr uint8_t IO_SET_LED = 80U;
constexpr uint8_t IO_SET_NEOPIXEL = 81U;
constexpr uint8_t IO_STATUS = 82U;

