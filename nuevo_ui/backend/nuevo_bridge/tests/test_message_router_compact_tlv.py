import ctypes

from nuevo_bridge.TLV_TypeDefs import DC_ENABLE, SENSOR_VOLTAGE, SYS_STATUS
from nuevo_bridge.message_router import MessageRouter
from nuevo_bridge.payloads import PayloadDCEnable, PayloadSensorVoltage, PayloadSystemStatus
from tlvcodec import DecodeErrorCode, Decoder, Encoder


class _DummyWsManager:
    connections = []


def main() -> None:
    router = MessageRouter(_DummyWsManager())
    messages = []

    def callback(error_code, frame_header, tlv_list):
        assert error_code == DecodeErrorCode.NoError
        assert frame_header.numTlvs == 2
        for tlv_type, _tlv_len, tlv_data in tlv_list:
            decoded = router.decode_incoming(tlv_type, tlv_data)
            assert decoded is not None
            messages.append(decoded)

    status = PayloadSystemStatus()
    status.firmwareMajor = 0
    status.firmwareMinor = 9
    status.firmwarePatch = 0
    status.state = 2
    status.uptimeMs = 1234
    status.lastRxMs = 20
    status.lastCmdMs = 25
    status.batteryMv = 12100
    status.rail5vMv = 5000
    status.errorFlags = 0
    status.attachedSensors = 0x01
    status.freeSram = 1500
    status.loopTimeAvgUs = 200
    status.loopTimeMaxUs = 400
    status.uartRxErrors = 0
    status.motorDirMask = 0
    status.neoPixelCount = 1
    status.heartbeatTimeoutMs = 500
    status.limitSwitchMask = 0
    for i in range(4):
        status.stepperHomeLimitGpio[i] = 0xFF

    voltage = PayloadSensorVoltage()
    voltage.batteryMv = 12100
    voltage.rail5vMv = 5000
    voltage.servoRailMv = 6000
    voltage.reserved = 0

    encoder = Encoder(deviceId=1, crc=True)
    encoder.addPacket(SYS_STATUS, ctypes.sizeof(status), status)
    encoder.addPacket(SENSOR_VOLTAGE, ctypes.sizeof(voltage), voltage)
    length, buffer = encoder.wrapupBuffer()

    decoder = Decoder(callback=callback, crc=True)
    decoder.decode(buffer[:length])

    assert len(messages) == 2
    assert messages[0]["topic"] == "system_status"
    assert messages[0]["data"]["firmwareMinor"] == 9
    assert messages[0]["data"]["heartbeatTimeoutMs"] == 500
    assert messages[1]["topic"] == "voltage"
    assert messages[1]["data"]["servoRailMv"] == 6000

    outgoing = router.handle_outgoing("dc_enable", {"motorNumber": 1, "mode": 2})
    assert outgoing is not None
    tlv_type, payload = outgoing
    assert tlv_type == DC_ENABLE
    assert isinstance(payload, PayloadDCEnable)
    assert payload.motorId == 0
    assert payload.mode == 2

    print("PASS: message router compact tlv")


if __name__ == "__main__":
    main()
