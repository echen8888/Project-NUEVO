# TLV Payload Specifications

**Version:** 3.0
**Date:** 2026-02-19

This document defines the packed binary payload for every TLV message type.
Type ID constants are defined in `TLV_TypeDefs.json` and auto-generated into:
- `firmware/arduino/src/messages/TLV_TypeDefs.h` (C++)
- `nuevo_ui/backend/nuevo_bridge/TLV_TypeDefs.py` (Python)

Payload struct definitions are mirrored in:
- `firmware/arduino/src/messages/TLV_Payloads.h` (C++)
- `nuevo_ui/backend/nuevo_bridge/payloads.py` (Python `ctypes`)

See `docs/COMMUNICATION_PROTOCOL.md` for the full system design including state
machine, safety design, and streaming rates. The code files above are the
practical reference for field layout; this document defines the wire framing,
message families, and payload conventions.

---

## Frame Structure

Each UART transmission is a **frame** consisting of a 12-byte `FrameHeader`
followed by one or more TLV packets:

```
FrameHeader (12 bytes):
  uint8_t  magicNum[4]    — sync pattern for frame boundary detection
  uint16_t numTotalBytes  — total byte count of this frame
  uint16_t checksum       — CRC16-CCITT over bytes 8+ (deviceId onward + TLVs)
  uint8_t  deviceId       — sender ID
  uint8_t  frameNum       — incrementing frame sequence number
  uint8_t  numTlvs        — number of TLV packets that follow
  uint8_t  flags          — bit0 = CRC16 enabled

Per TLV packet (2-byte header + payload):
  uint8_t  tlvType        — message type ID (from TLV_TypeDefs.json)
  uint8_t  tlvLen         — payload byte count (0–255)
  uint8_t  payload[tlvLen]
```

The CRC16 covers the frame starting at byte 8 (skipping `magicNum`,
`numTotalBytes`, and `checksum` itself). Defined by `CRC16_BYTES2IGNORE = 8`
in `tlvcodec.h`.

Frame overhead per message: **14 bytes** (12-byte frame header + 2-byte TLV header).
Multiple TLVs can share one FrameHeader (see bundling note in
`docs/COMMUNICATION_PROTOCOL.md` Section 8.2).

## Payload Conventions

- All multi-byte integers are **little-endian**.
- All structs are **tightly packed** — no padding bytes.
  - C++: `#pragma pack(push, 1)` ... `#pragma pack(pop)`
  - Python: use `struct.pack` with `<` (little-endian) prefix.
- `float` is IEEE 754 single precision (4 bytes).
- Direction: ↓ = RPi → Arduino, ↑ = Arduino → RPi.
- **Payload size** is the byte count of the TLV payload struct only
  (excludes `FrameHeader` and `TlvHeader` overhead).

---

## System Messages (types 1–5)

---

### `SYS_HEARTBEAT = 1` ↓

Sent by the Bridge to keep the Arduino liveness timer alive. Any received TLV
resets the timer; the Bridge sends this when no other message is pending.
Target rate: 200 ms interval (5 Hz). Arduino motor timeout: 500 ms (configurable).

| Field | Type | Size | Description |
|---|---|---|---|
| `timestamp` | `uint32_t` | 4 | RPi milliseconds since boot |
| `flags` | `uint8_t` | 1 | Reserved (set to 0) |

**Total payload: 5 bytes**

C++ format string: `<IB`

---

### `SYS_STATUS = 2` ↑

Periodic system health report from Arduino.
Rate: 1 Hz (IDLE / E-STOP), 10 Hz (RUNNING / ERROR).

| Field | Type | Size | Description |
|---|---|---|---|
| `firmwareMajor` | `uint8_t` | 1 | Firmware major version |
| `firmwareMinor` | `uint8_t` | 1 | Firmware minor version |
| `firmwarePatch` | `uint8_t` | 1 | Firmware patch version |
| `state` | `uint8_t` | 1 | System state: 0=INIT, 1=IDLE, 2=RUNNING, 3=ERROR, 4=ESTOP |
| `uptimeMs` | `uint32_t` | 4 | Arduino uptime (ms) |
| `lastRxMs` | `uint32_t` | 4 | ms since last TLV received from RPi |
| `lastCmdMs` | `uint32_t` | 4 | ms since last non-heartbeat command received |
| `batteryMv` | `uint16_t` | 2 | Battery voltage (mV) |
| `rail5vMv` | `uint16_t` | 2 | 5V regulated rail voltage (mV) |
| `errorFlags` | `uint8_t` | 1 | Error bitmask (see below) |
| `attachedSensors` | `uint8_t` | 1 | Sensor bitmask: bit0=IMU, bit1=Lidar, bit2=Ultrasonic |
| `freeSram` | `uint16_t` | 2 | Free SRAM (bytes) |
| `loopTimeAvgUs` | `uint16_t` | 2 | Average control loop time (µs) |
| `loopTimeMaxUs` | `uint16_t` | 2 | Maximum control loop time since last status (µs) |
| `uartRxErrors` | `uint16_t` | 2 | Cumulative UART CRC/framing errors |
| `wheelDiameterMm` | `float` | 4 | Configured wheel diameter (mm) |
| `wheelBaseMm` | `float` | 4 | Configured wheel base center-to-center (mm) |
| `motorDirMask` | `uint8_t` | 1 | Direction inversion bitmask (bit N = motor N inverted) |
| `neoPixelCount` | `uint8_t` | 1 | Configured NeoPixel count |
| `heartbeatTimeoutMs` | `uint16_t` | 2 | Configured liveness timeout (ms) |
| `limitSwitchMask` | `uint16_t` | 2 | Bitmask: bit N = GPIO N is configured as a limit switch |
| `stepperHomeLimitGpio[4]` | `uint8_t[4]` | 4 | GPIO index used as home limit for each stepper; 0xFF = none |

**Total payload: 54 bytes**

Python format string: `<BBBBIIIHHBBHHHHffBBHH4B`

> `limitSwitchMask` and `stepperHomeLimitGpio` are **compile-time constants**
> from `config.h`. They cannot be changed at runtime. These fields let the
> Bridge and UI correctly label which bits in `IO_STATUS.buttonMask` are
> limit switch events and which stepper each one belongs to.

**`errorFlags` bitmask:**
| Bit | Constant | Condition |
|---|---|---|
| 0 | `ERR_UNDERVOLTAGE` | Battery voltage below minimum threshold |
| 1 | `ERR_OVERVOLTAGE` | Battery voltage above maximum threshold |
| 2 | `ERR_ENCODER_FAIL` | Encoder reads 0 for >500 ms while PWM > 20% |
| 3 | `ERR_I2C_ERROR` | I2C bus error (PCA9685 or IMU not responding) |
| 4 | `ERR_IMU_ERROR` | IMU specifically not responding or DMP failure |
| 5 | `ERR_LIVENESS_LOST` | Liveness timeout triggered (motors cut, not in error state) |
| 6 | `ERR_LOOP_OVERRUN` | Control loop exceeded deadline in last reporting period |

---

### `SYS_CMD = 3` ↓

State machine control commands. Accepted in all states except E-STOP.
(E-STOP ignores all incoming TLV.)

| Field | Type | Size | Description |
|---|---|---|---|
| `command` | `uint8_t` | 1 | Command ID (see below) |
| `reserved` | `uint8_t[3]` | 3 | Set to 0 |

**Total payload: 4 bytes**

Python format string: `<B3x`

**Command IDs:**
| Value | Constant | Action |
|---|---|---|
| 1 | `SYS_CMD_START` | IDLE → RUNNING |
| 2 | `SYS_CMD_STOP` | RUNNING → IDLE (disables all actuators gracefully) |
| 3 | `SYS_CMD_RESET` | ERROR → IDLE, or **E-STOP → IDLE** |
| 4 | `SYS_CMD_ESTOP` | Any state → E-STOP |

---

### `SYS_CONFIG = 4` ↓

Configure system parameters. **IDLE state only.** Ignored in RUNNING.
Fields with sentinel value (0 or 0xFF) are not changed.

| Field | Type | Size | Description |
|---|---|---|---|
| `wheelDiameterMm` | `float` | 4 | Wheel diameter (mm); 0.0 = no change |
| `wheelBaseMm` | `float` | 4 | Wheel base (mm); 0.0 = no change |
| `motorDirMask` | `uint8_t` | 1 | Direction inversion bitmask to apply |
| `motorDirChangeMask` | `uint8_t` | 1 | Which motors to update direction for (bitmask) |
| `neoPixelCount` | `uint8_t` | 1 | NeoPixel count; 0 = no change |
| `attachedSensors` | `uint8_t` | 1 | Sensor bitmask; 0xFF = no change |
| `heartbeatTimeoutMs` | `uint16_t` | 2 | Liveness timeout (ms); 0 = no change |
| `resetOdometry` | `uint8_t` | 1 | 1 = reset x, y, theta to (0, 0, 0) |
| `reserved` | `uint8_t` | 1 | Set to 0 |

**Total payload: 16 bytes**

Python format string: `<ffBBBBHBx`

---

### `SYS_SET_PID = 5` ↓

Set PID gains for one DC motor loop.
Accepted in **IDLE and RUNNING** states (live tuning supported).

| Field | Type | Size | Description |
|---|---|---|---|
| `motorId` | `uint8_t` | 1 | DC motor index (0–3) |
| `loopType` | `uint8_t` | 1 | 0 = position loop, 1 = velocity loop |
| `reserved` | `uint8_t[2]` | 2 | Set to 0 |
| `kp` | `float` | 4 | Proportional gain |
| `ki` | `float` | 4 | Integral gain |
| `kd` | `float` | 4 | Derivative gain |
| `maxOutput` | `float` | 4 | Output clamp (default: 255.0) |
| `maxIntegral` | `float` | 4 | Anti-windup integral limit |

**Total payload: 24 bytes**

Python format string: `<BB2xfffff`

---

## DC Motor Messages (types 16–20)

---

### `DC_ENABLE = 256` ↓

Enable or disable a DC motor and select its control mode.
Disabling coasts the motor (PWM = 0, H-bridge disabled).

| Field | Type | Size | Description |
|---|---|---|---|
| `motorId` | `uint8_t` | 1 | Motor index (0–3) |
| `mode` | `uint8_t` | 1 | 0=disable, 1=position, 2=velocity, 3=pwm |
| `reserved` | `uint8_t[2]` | 2 | Set to 0 |

**Total payload: 4 bytes**

Python format string: `<BB2x`

**Control modes:**
| Value | Mode | Description |
|---|---|---|
| 0 | Disabled | H-bridge off, motor coasts |
| 1 | Position | Cascade PID: outer position loop, inner velocity loop |
| 2 | Velocity | Velocity PID loop |
| 3 | PWM | Direct PWM output (open loop) |

---

### `DC_SET_POSITION = 257` ↓

Set absolute target position for a motor in position control mode.
Only effective when motor is in position mode (mode=1).

| Field | Type | Size | Description |
|---|---|---|---|
| `motorId` | `uint8_t` | 1 | Motor index (0–3) |
| `reserved` | `uint8_t[3]` | 3 | Set to 0 |
| `targetTicks` | `int32_t` | 4 | Target position (encoder ticks) |
| `maxVelTicks` | `int32_t` | 4 | Velocity cap during move (ticks/sec); 0 = use default |

**Total payload: 12 bytes**

Python format string: `<B3xii`

---

### `DC_SET_VELOCITY = 258` ↓

Set target velocity for a motor in velocity control mode.
Only effective when motor is in velocity mode (mode=2).

| Field | Type | Size | Description |
|---|---|---|---|
| `motorId` | `uint8_t` | 1 | Motor index (0–3) |
| `reserved` | `uint8_t[3]` | 3 | Set to 0 |
| `targetTicks` | `int32_t` | 4 | Target velocity (ticks/sec); negative = reverse |

**Total payload: 8 bytes**

Python format string: `<B3xi`

---

### `DC_SET_PWM = 259` ↓

Set direct PWM output for a motor in PWM control mode.
Only effective when motor is in PWM mode (mode=3).

| Field | Type | Size | Description |
|---|---|---|---|
| `motorId` | `uint8_t` | 1 | Motor index (0–3) |
| `reserved` | `uint8_t` | 1 | Set to 0 |
| `pwm` | `int16_t` | 2 | PWM value: -255 to +255 (sign determines direction) |

**Total payload: 4 bytes**

Python format string: `<Bbh` → note: use `<BBh` with unsigned motor ID

---

### `DC_STATUS_ALL = 260` ↑

Status for all 4 DC motors in one packed message.
Rate: **100 Hz** in RUNNING state.

One `DCMotorStatus` struct per motor, motors 0–3 in order (no motor ID field needed).

**`DCMotorStatus` struct (46 bytes each):**

| Field | Type | Size | Description |
|---|---|---|---|
| `mode` | `uint8_t` | 1 | Current control mode (0=disabled, 1=pos, 2=vel, 3=pwm) |
| `faultFlags` | `uint8_t` | 1 | bit0=overcurrent, bit1=stall |
| `position` | `int32_t` | 4 | Current position (encoder ticks) — closed-loop measurement |
| `velocity` | `int32_t` | 4 | Current velocity (ticks/sec) |
| `targetPos` | `int32_t` | 4 | Current target position |
| `targetVel` | `int32_t` | 4 | Current target velocity |
| `pwmOutput` | `int16_t` | 2 | Actual PWM output (-255 to +255) |
| `currentMa` | `int16_t` | 2 | Motor current (mA); -1 if not measured |
| `posKp` | `float` | 4 | Position loop Kp |
| `posKi` | `float` | 4 | Position loop Ki |
| `posKd` | `float` | 4 | Position loop Kd |
| `velKp` | `float` | 4 | Velocity loop Kp |
| `velKi` | `float` | 4 | Velocity loop Ki |
| `velKd` | `float` | 4 | Velocity loop Kd |

**Per-motor size: 2 + 4×4 + 2×2 + 6×4 = 46 bytes**
**Total payload: 4 × 46 = 184 bytes**

Python format string (one motor): `<BBiiiihh6f`
Python format string (all 4 motors): `<` + `BBiiiihh6f` × 4

---

## Stepper Motor Messages (types 32–36)

---

### `STEP_ENABLE = 512` ↓

Enable or disable a stepper motor driver (ENABLE pin).

| Field | Type | Size | Description |
|---|---|---|---|
| `stepperId` | `uint8_t` | 1 | Stepper index (0–3) |
| `enable` | `uint8_t` | 1 | 0 = disable (coil off, motor free), 1 = enable (coil on, holds position) |
| `reserved` | `uint8_t[2]` | 2 | Set to 0 |

**Total payload: 4 bytes**

Python format string: `<BB2x`

---

### `STEP_SET_PARAMS = 513` ↓

Set motion parameters for a stepper. Can be sent before or during a move.
If sent during a move, parameters take effect on the next move command.

| Field | Type | Size | Description |
|---|---|---|---|
| `stepperId` | `uint8_t` | 1 | Stepper index (0–3) |
| `reserved` | `uint8_t[3]` | 3 | Set to 0 |
| `maxVelocity` | `uint32_t` | 4 | Maximum speed (steps/sec) |
| `acceleration` | `uint32_t` | 4 | Acceleration and deceleration (steps/sec²) |

**Total payload: 12 bytes**

Python format string: `<B3xII`

---

### `STEP_MOVE = 514` ↓

Command a stepper move. Motor must be enabled first.

| Field | Type | Size | Description |
|---|---|---|---|
| `stepperId` | `uint8_t` | 1 | Stepper index (0–3) |
| `moveType` | `uint8_t` | 1 | 0 = absolute position, 1 = relative steps |
| `reserved` | `uint8_t[2]` | 2 | Set to 0 |
| `target` | `int32_t` | 4 | Target step count (absolute) or step delta (relative) |

**Total payload: 8 bytes**

Python format string: `<BB2xi`

---

### `STEP_HOME = 515` ↓

Run homing sequence: move until a limit switch triggers, then zero the position.

| Field | Type | Size | Description |
|---|---|---|---|
| `stepperId` | `uint8_t` | 1 | Stepper index (0–3) |
| `direction` | `int8_t` | 1 | -1 = reverse, +1 = forward |
| `reserved` | `uint8_t[2]` | 2 | Set to 0 |
| `homeVelocity` | `uint32_t` | 4 | Homing speed (steps/sec); should be slow |
| `backoffSteps` | `int32_t` | 4 | Steps to back off after limit triggered |

**Total payload: 12 bytes**

Python format string: `<Bb2xIi`

---

### `STEP_STATUS_ALL = 516` ↑

Status for all 4 steppers in one packed message.
Rate: **100 Hz** in RUNNING state.

One `StepperStatus` struct per stepper, steppers 0–3 in order.

> **Important:** `commandedCount` is the firmware's internal step counter.
> Steppers are **open-loop** — this value assumes no missed steps.
> It is **not** a measured position. Do not label it "current position" in the UI.

**`StepperStatus` struct (24 bytes each):**

| Field | Type | Size | Description |
|---|---|---|---|
| `enabled` | `uint8_t` | 1 | 0 = disabled, 1 = enabled |
| `motionState` | `uint8_t` | 1 | 0=idle, 1=accel, 2=cruise, 3=decel, 4=homing, 5=fault |
| `limitHit` | `uint8_t` | 1 | bit0 = min limit triggered, bit1 = max limit triggered |
| `reserved` | `uint8_t` | 1 | |
| `commandedCount` | `int32_t` | 4 | Commanded step count (open-loop, not measured) |
| `targetCount` | `int32_t` | 4 | Current move target |
| `currentSpeed` | `uint32_t` | 4 | Current speed (steps/sec) |
| `maxSpeed` | `uint32_t` | 4 | Configured max speed |
| `acceleration` | `uint32_t` | 4 | Configured acceleration |

**Per-stepper size: 4 + 3×4 + 2×4 = 24 bytes**
**Total payload: 4 × 24 = 96 bytes**

Python format string (one stepper): `<BBBxiiIII`
Python format string (all 4 steppers): `<` + `BBBxiiIII` × 4

---

## Servo Messages (types 48–50)

---

### `SERVO_ENABLE = 768` ↓

Enable or disable individual servo channels. All channels start disabled on reset.
Disabling a channel cuts the PWM signal (servo goes limp / relaxes).
The PCA9685 controller itself is always powered when connected.

| Field | Type | Size | Description |
|---|---|---|---|
| `channel` | `uint8_t` | 1 | Servo channel (0–15); **0xFF = all channels** |
| `enable` | `uint8_t` | 1 | 0 = disable (no pulse), 1 = enable |
| `reserved` | `uint8_t[2]` | 2 | Set to 0 |

**Total payload: 4 bytes**

Python format string: `<BB2x`

---

### `SERVO_SET = 769` ↓

Set servo target position as pulse width. Two variants share the same type ID.
Determine which variant by the `count` field: count=1 for single, count>1 for bulk.

**Single servo:**

| Field | Type | Size | Description |
|---|---|---|---|
| `channel` | `uint8_t` | 1 | Servo channel (0–15) |
| `count` | `uint8_t` | 1 | Must be 1 |
| `pulseUs` | `uint16_t[1]` | 2 | Pulse width (µs), typically 500–2500 |

**Bulk update (consecutive channels):**

| Field | Type | Size | Description |
|---|---|---|---|
| `startChannel` | `uint8_t` | 1 | First channel to update |
| `count` | `uint8_t` | 1 | Number of channels (1–16) |
| `pulseUs[16]` | `uint16_t[16]` | 32 | Pulse widths; only first `count` values used |

For synchronized multi-servo motion, use bulk with all channels in one message.

**Single payload: 4 bytes** | **Bulk payload: 2 + 32 = 34 bytes**

Python format strings: `<BBH` (single), `<BB16H` (bulk)

---

### `SERVO_STATUS_ALL = 770` ↑

PCA9685 controller status and commanded positions for all 16 channels.
Rate: **50 Hz** in RUNNING state.

> Servo position feedback is not available. This reports commanded state only.
> On firmware reset, all pulse widths are 0 (disabled).

| Field | Type | Size | Description |
|---|---|---|---|
| `pca9685Connected` | `uint8_t` | 1 | 0 = not detected on I2C, 1 = connected |
| `pca9685Error` | `uint8_t` | 1 | PCA9685 error flags (0 = no error) |
| `enabledMask` | `uint16_t` | 2 | Bitmask of enabled channels (bit N = channel N) |
| `pulseUs[16]` | `uint16_t[16]` | 32 | Commanded pulse width per channel (0 = disabled) |

**Total payload: 36 bytes**

Python format string: `<BBH16H`

---

## Sensor Messages (types 64–69)

---

### `SENSOR_IMU = 1024` ↑

Full ICM-20948 sensor data.
Rate: **100 Hz** in RUNNING state (if IMU attached and configured).

Quaternion and global acceleration are computed by the **Fusion AHRS library**
(Madgwick-based, running on the Arduino). In 9-DOF mode the magnetometer is
fused; in 6-DOF fallback mode (mag uncalibrated or disabled) the `magCalibrated`
flag is 0 and yaw will drift. If the IMU is not connected, this TLV is not sent.

| Field | Type | Size | Description |
|---|---|---|---|
| `quatW` | `float` | 4 | Orientation quaternion W (Fusion AHRS output) |
| `quatX` | `float` | 4 | Orientation quaternion X |
| `quatY` | `float` | 4 | Orientation quaternion Y |
| `quatZ` | `float` | 4 | Orientation quaternion Z |
| `earthAccX` | `float` | 4 | Earth-frame linear accel X (g, gravity removed) |
| `earthAccY` | `float` | 4 | Earth-frame linear accel Y (g, gravity removed) |
| `earthAccZ` | `float` | 4 | Earth-frame linear accel Z (g, gravity removed) |
| `rawAccX` | `int16_t` | 2 | Accelerometer X in mg — cast of `accX()` (range ±16000 mg for ±16g) |
| `rawAccY` | `int16_t` | 2 | Accelerometer Y in mg |
| `rawAccZ` | `int16_t` | 2 | Accelerometer Z in mg |
| `rawGyroX` | `int16_t` | 2 | Gyroscope X in 0.1 DPS units — `gyrX() × 10` cast to int16 (range ±20000 for ±2000 DPS) |
| `rawGyroY` | `int16_t` | 2 | Gyroscope Y in 0.1 DPS units |
| `rawGyroZ` | `int16_t` | 2 | Gyroscope Z in 0.1 DPS units |
| `magX` | `int16_t` | 2 | Magnetometer X in µT — cast of `magX()` (calibration offset already applied) |
| `magY` | `int16_t` | 2 | Magnetometer Y in µT |
| `magZ` | `int16_t` | 2 | Magnetometer Z in µT |
| `magCalibrated` | `uint8_t` | 1 | 0 = no mag calibration (6-DOF fusion), 1 = calibrated (9-DOF fusion) |
| `reserved` | `uint8_t` | 1 | Set to 0 |
| `timestamp` | `uint32_t` | 4 | Arduino `micros()` at sample time |

> `rawAccX/Y/Z`: `(int16_t)myICM.accX()` — `accX()` already returns mg as float.
> `rawGyroX/Y/Z`: `(int16_t)(myICM.gyrX() * 10)` — 0.1 DPS resolution.
> `magX/Y/Z`: `(int16_t)myICM.magX()` with calibration offset subtracted.

**Total payload: 7×4 + 9×2 + 2×1 + 4 = 28 + 18 + 2 + 4 = 52 bytes**

Python format string: `<7f9hBBI`

---

### `SENSOR_KINEMATICS = 1025` ↑

Computed robot pose and velocity from wheel encoder odometry.
Rate: **100 Hz** in RUNNING state.

Computed using configured `wheelDiameterMm` and `wheelBaseMm`.
Position resets to (0, 0, 0) on firmware reset or `SYS_CONFIG(resetOdometry=1)`.

| Field | Type | Size | Description |
|---|---|---|---|
| `x` | `float` | 4 | Position X from start (mm) |
| `y` | `float` | 4 | Position Y from start (mm) |
| `theta` | `float` | 4 | Heading (radians, CCW positive from start) |
| `vx` | `float` | 4 | Velocity X in robot frame (mm/s) |
| `vy` | `float` | 4 | Velocity Y in robot frame (mm/s) |
| `vTheta` | `float` | 4 | Angular velocity (rad/s, CCW positive) |
| `timestamp` | `uint32_t` | 4 | Arduino `micros()` at compute time |

**Total payload: 6×4 + 4 = 28 bytes**

Python format string: `<6fI`

---

### `SENSOR_VOLTAGE = 1026` ↑

Battery and power rail voltage readings.
Rate: **10 Hz** in RUNNING and ERROR states.

| Field | Type | Size | Description |
|---|---|---|---|
| `batteryMv` | `uint16_t` | 2 | Battery input voltage (mV) |
| `rail5vMv` | `uint16_t` | 2 | 5V regulated rail voltage (mV) |
| `servoRailMv` | `uint16_t` | 2 | Servo power rail voltage (mV); 0 if jumper not connected |
| `reserved` | `uint16_t` | 2 | Reserved for future rail (set to 0) |

**Total payload: 8 bytes**

Python format string: `<4H`

**Battery voltage thresholds (default):**
| Threshold | Value | Error Flag |
|---|---|---|
| Minimum (undervoltage) | 10,000 mV (10.0 V) | `ERR_UNDERVOLTAGE` |
| Maximum (overvoltage) | 25,000 mV (25.0 V) | `ERR_OVERVOLTAGE` |

---

### `SENSOR_RANGE = 1027` ↑

Distance measurement from one range sensor. One message per sensor.
Rate: depends on sensor polling frequency (typically 10–50 Hz for ultrasonic).

| Field | Type | Size | Description |
|---|---|---|---|
| `sensorId` | `uint8_t` | 1 | Sensor index (0-based, in order of configuration) |
| `sensorType` | `uint8_t` | 1 | 0 = ultrasonic, 1 = lidar |
| `status` | `uint8_t` | 1 | 0 = valid reading, 1 = out of range, 2 = sensor error |
| `reserved` | `uint8_t` | 1 | Set to 0 |
| `distanceMm` | `uint16_t` | 2 | Measured distance (mm); 0 if status ≠ 0 |
| `reserved2` | `uint16_t` | 2 | Reserved |
| `timestamp` | `uint32_t` | 4 | Arduino `micros()` at measurement time |

**Total payload: 12 bytes**

Python format string: `<BBBBHhI` → or `<4BHxI` (cleaner)

---

### `SENSOR_MAG_CAL_CMD = 1028` ↓

Initiate, stop, or apply magnetometer calibration.
**IDLE state only** — ignored in RUNNING. Calibration requires the user to rotate
the robot through full 3D orientations while this mode is active.

| Field | Type | Size | Description |
|---|---|---|---|
| `command` | `uint8_t` | 1 | Command (see below) |
| `reserved` | `uint8_t[3]` | 3 | Set to 0 |
| `offsetX` | `float` | 4 | Hard-iron offset X (µT); used only with `CMD_APPLY` |
| `offsetY` | `float` | 4 | Hard-iron offset Y (µT) |
| `offsetZ` | `float` | 4 | Hard-iron offset Z (µT) |

**Total payload: 16 bytes**

Python format string: `<B3xfff`

**Command IDs:**
| Value | Constant | Action |
|---|---|---|
| 1 | `MAG_CAL_START` | Start calibration sampling; Arduino streams `SENSOR_MAG_CAL_STATUS` at ~10 Hz |
| 2 | `MAG_CAL_STOP` | Stop sampling without saving |
| 3 | `MAG_CAL_SAVE` | Save current computed offsets to EEPROM and activate 9-DOF mode |
| 4 | `MAG_CAL_APPLY` | Apply user-provided offsets (from payload) and save to EEPROM; skips sampling |
| 5 | `MAG_CAL_CLEAR` | Clear EEPROM calibration; revert to 6-DOF (uncalibrated) mode |

> During `MAG_CAL_START` the robot should remain **stationary except for slow
> rotation**. The Arduino collects min/max on each axis and computes hard-iron
> offsets as (max+min)/2. A minimum of 50 samples is required before `MAG_CAL_SAVE`
> is accepted; the status message reports the current sample count.

---

### `SENSOR_MAG_CAL_STATUS = 1029` ↑

Magnetometer calibration progress report.
Sent at **~10 Hz** while calibration is active (`MAG_CAL_START` received).
Also sent once when calibration completes or is cancelled.

| Field | Type | Size | Description |
|---|---|---|---|
| `state` | `uint8_t` | 1 | 0=idle, 1=sampling, 2=complete, 3=saved, 4=error |
| `sampleCount` | `uint16_t` | 2 | Number of magnetometer samples collected |
| `reserved` | `uint8_t` | 1 | Set to 0 |
| `minX` | `float` | 4 | Current minimum magnetometer X seen (µT) |
| `maxX` | `float` | 4 | Current maximum magnetometer X seen (µT) |
| `minY` | `float` | 4 | Current minimum magnetometer Y seen (µT) |
| `maxY` | `float` | 4 | Current maximum magnetometer Y seen (µT) |
| `minZ` | `float` | 4 | Current minimum magnetometer Z seen (µT) |
| `maxZ` | `float` | 4 | Current maximum magnetometer Z seen (µT) |
| `offsetX` | `float` | 4 | Computed hard-iron offset X (µT) = (maxX+minX)/2 |
| `offsetY` | `float` | 4 | Computed hard-iron offset Y (µT) |
| `offsetZ` | `float` | 4 | Computed hard-iron offset Z (µT) |
| `savedToEeprom` | `uint8_t` | 1 | 1 = offsets currently loaded from / saved to EEPROM |
| `reserved2` | `uint8_t[3]` | 3 | Set to 0 |

**Total payload: 4 + 9×4 + 4 = 44 bytes**

Python format string: `<BHx9fB3x`

> If EEPROM is available (Arduino Mega 2560: 4 KB), offsets are persisted across
> power cycles and automatically loaded on startup. If no saved calibration exists,
> the firmware starts in 6-DOF mode and sets `magCalibrated=0` in `SENSOR_IMU`.
> The UI should warn the user to calibrate if `magCalibrated=0` in RUNNING state.

---

## User I/O Messages (types 80–82)

---

### `IO_SET_LED = 1280` ↓

Set the state of a user LED or the status LED.

| Field | Type | Size | Description |
|---|---|---|---|
| `ledId` | `uint8_t` | 1 | LED index (see firmware `pins.h`) |
| `mode` | `uint8_t` | 1 | 0=off, 1=on, 2=blink, 3=breathe*, 4=pwm |
| `brightness` | `uint8_t` | 1 | Brightness 0–255 (for PWM / breathe modes) |
| `reserved` | `uint8_t` | 1 | Set to 0 |
| `periodMs` | `uint16_t` | 2 | Blink / breathe period (ms) |
| `dutyCycle` | `uint16_t` | 2 | Blink on-time: 0–1000 represents 0.0–100.0% |

**Total payload: 8 bytes**

Python format string: `<BBBxHH`

> *LED_RED (pin 5, Rev. B) cannot use breathe mode — Timer3 conflict with stepper
> pulse generation. The firmware automatically falls back to blink mode for this LED.
> See `firmware/TIMER3_CONFLICT_ANALYSIS.md` for details.

---

### `IO_SET_NEOPIXEL = 1281` ↓

Set color for one NeoPixel or all NeoPixels.

| Field | Type | Size | Description |
|---|---|---|---|
| `index` | `uint8_t` | 1 | Pixel index (0-based); **0xFF = all pixels** |
| `red` | `uint8_t` | 1 | Red component (0–255) |
| `green` | `uint8_t` | 1 | Green component (0–255) |
| `blue` | `uint8_t` | 1 | Blue component (0–255) |

**Total payload: 4 bytes**

Python format string: `<BBBB`

---

### `IO_STATUS = 1282` ↑

All UserIO state in one message.
Rate: **100 Hz** in RUNNING state.

**Button and limit switch sharing:** All digital input GPIOs (buttons and
limit switches) share the same physical pins. A single `buttonMask` covers
all of them. The Bridge uses `SYS_STATUS.limitSwitchMask` and
`stepperHomeLimitGpio[]` to know which bits to display as limit switch events
vs. user button presses. Students can trigger limit switch behavior by
physically pressing the corresponding button — no external switch needed.

| Field | Type | Size | Description |
|---|---|---|---|
| `buttonMask` | `uint16_t` | 2 | Bitmask of all digital input GPIOs (buttons + limit switches combined) |
| `ledBrightness[3]` | `uint8_t[3]` | 3 | Brightness of each user LED (0=off, 255=full) |
| `reserved` | `uint8_t` | 1 | Set to 0 |
| `timestamp` | `uint32_t` | 4 | Arduino `millis()` |
| `neoPixels[n×3]` | `uint8_t[]` | variable | RGB triplets for each NeoPixel; length = `neoPixelCount × 3` |

**Fixed payload: 10 bytes + 3 × neoPixelCount bytes**

Python format string (fixed part): `<H3BxI`
Append `neoPixelCount × 3` bytes of NeoPixel RGB data after the fixed fields.

---

## TLV Type ID Quick Reference

| Range | Category |
|---|---|
| 1–19 | System |
| 256–279 | DC Motors |
| 512–539 | Stepper Motors |
| 768–789 | Servos |
| 1024–1059 | Sensors |
| 1280–1299 | User I/O |

| ID | Name | Dir | Payload (bytes) |
|---|---|---|---|
| 1 | `SYS_HEARTBEAT` | ↓ | 5 |
| 2 | `SYS_STATUS` | ↑ | 54 |
| 3 | `SYS_CMD` | ↓ | 4 |
| 4 | `SYS_CONFIG` | ↓ | 16 |
| 5 | `SYS_SET_PID` | ↓ | 24 |
| 256 | `DC_ENABLE` | ↓ | 4 |
| 257 | `DC_SET_POSITION` | ↓ | 12 |
| 258 | `DC_SET_VELOCITY` | ↓ | 8 |
| 259 | `DC_SET_PWM` | ↓ | 4 |
| 260 | `DC_STATUS_ALL` | ↑ | 184 |
| 512 | `STEP_ENABLE` | ↓ | 4 |
| 513 | `STEP_SET_PARAMS` | ↓ | 12 |
| 514 | `STEP_MOVE` | ↓ | 8 |
| 515 | `STEP_HOME` | ↓ | 12 |
| 516 | `STEP_STATUS_ALL` | ↑ | 96 |
| 768 | `SERVO_ENABLE` | ↓ | 4 |
| 769 | `SERVO_SET` | ↓ | 4 or 34 |
| 770 | `SERVO_STATUS_ALL` | ↑ | 36 |
| 1024 | `SENSOR_IMU` | ↑ | 52 |
| 1025 | `SENSOR_KINEMATICS` | ↑ | 28 |
| 1026 | `SENSOR_VOLTAGE` | ↑ | 8 |
| 1027 | `SENSOR_RANGE` | ↑ | 12 |
| 1028 | `SENSOR_MAG_CAL_CMD` | ↓ | 16 |
| 1029 | `SENSOR_MAG_CAL_STATUS` | ↑ | 44 |
| 1280 | `IO_SET_LED` | ↓ | 8 |
| 1281 | `IO_SET_NEOPIXEL` | ↓ | 4 |
| 1282 | `IO_STATUS` | ↑ | 10 + 3n |
