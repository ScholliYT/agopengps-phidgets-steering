import logging
import time
import socket
import struct
import threading

from agopengps_phidgets_steering.steering_controller import SteeringController


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(levelname)-8s %(name)-15s %(message)s"
)

# These are some magic numbers that AgIO uses to identify the data it sends and receives
# A reference can be found in the AgIO source code and documentation
# look at: https://github.com/AgOpenGPS-Official/Boards/blob/main/PGN.md
SOURCE_AGIO = 0x7F
SOURCE_AUTOSTEER = 0x7E  # this is us, the autosteer controller

# packets coming from AgIO
PGN_AUTOSTEER_DATA = 0xFE  # 254
PGN_STEER_SETTINGS = 0xFC  # 252
PGN_STEER_CONFIG = 0xFB  # 251
PGN_HELLO_REQUEST = 0xC8  # 200
# PGN_SUBNET_SET = 0xC9 # 201
PGN_SUBNET_SCAN_REQUEST = 0xCA  # 202
PGN_SUBNET_SCAN_REPLY = 0xCB  # 203

# packets going to AgIO
# this is what we send back to AgIO, containing the actual steering angle
PGN_DATA_FROM_AUTOSTEER = 0xFD  # 253
PGN_DATA_FROM_AUTOSTEER_2 = 0xFA  # 250
PGN_HELLO_REPLY_STEERING_1 = 0x7E  # AngleLo	AngleHi	CountsLo	CountsHi	Switchbyte  CRC
PGN_HELLO_REPLY_STEERING_2 = 0x7B  # relayLo	relayHi	*	        *	        *	        CRC


AGIO_NETWORK_IP = "192.168.5.255"
AGIO_RECEIVE_PORT = 9999  # Port that AgIO listens on
WAS_REPORTING_FREQUENCY = 40.0  # Hz


class AgIOAutsteer:
    def __init__(self, mc: SteeringController):
        self.logger = logging.getLogger(name="AgIOAutsteer")

        self.mc = mc

        # Client to send messages back to AgIO
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.client.settimeout(0)
        self.client.setblocking(0)

        # Server to receive messages from AgIO
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.server.bind(("", 8888))
        self.server.settimeout(0.25)

        self.server_running = threading.Event()
        self.server_running.set()
        self.server_thread = threading.Thread(target=self.server_loop)
        self.server_thread.start()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self.logger.info("Closing receive server thread")
        self.server_running.clear()
        self.server_thread.join(0.5)

        self.logger.info("Closing server and client UDP sockets")
        self.server.close()
        self.client.close()

    def server_loop(self) -> None:
        while self.server_running.is_set():
            try:
                while self.server_running.is_set():
                    (data, (src_ip, src_port)) = self.server.recvfrom(1024)
                    if data[0] == 0x80 and data[1] == 0x81:
                        self.decode_data(data, src_ip, src_port)
            except socket.timeout as e:
                self.logger.exception("Timeout error reading UDP data from AgIO")
            except socket.error as e:
                self.logger.exception("Error reading UDP data from AgIO")
        self.logger.info("Exiting server loop")

    def decode_data(self, data, src_ip: str, src_port: int) -> None:
        data_source = data[2]
        pgn_id = data[3]
        payload_length = data[4]

        if len(data) - 6 != payload_length:
            self.logger.warning(
                "Received data with invalid length: %s (expected %d, got %d)",
                data,
                payload_length,
                len(data) - 6,
            )
            return

        received_crc = data[-1]
        calculated_crc = self.calc_crc(data[:-1])

        if received_crc != calculated_crc:
            self.logger.warning(
                "Received data with invalid crc: %s (got %s, calculated crc: %s)",
                data,
                received_crc,
                calculated_crc,
            )
            return

        if data_source != SOURCE_AGIO:
            self.logger.info("Received data from unknown source %d", data_source)
            return

        # Autosteer data from AgIO
        if pgn_id == PGN_AUTOSTEER_DATA:
            payload = data[5:-1]
            unpacked_payload = {
                "Speed": struct.unpack("<H", payload[0:2])[0] / 10.0,
                "AutosteerActive": payload[2],
                "SteerAngle": struct.unpack("<h", payload[3:5])[0] / 100.0,
                "SectionControl": struct.unpack("<H", payload[6:8])[0],
            }
            self.logger.info(
                "Received AutoSteer data from AgIO containing { Speed %.1f, Steer angle %.2f, Autosteer %s, Section control %d}.",
                unpacked_payload["Speed"],
                unpacked_payload["SteerAngle"],
                "active" if unpacked_payload["AutosteerActive"] else "inactive",
                unpacked_payload["SectionControl"],
            )
            mc.target_angle = float(unpacked_payload["SteerAngle"])

            if not self.mc.steering_active.is_set() and unpacked_payload["AutosteerActive"]:
                self.logger.info("Activating motor for auto steering")
                self.mc.steering_active.set()
            elif self.mc.steering_active.is_set() and not unpacked_payload["AutosteerActive"]:
                self.logger.info("Deactivating motor for auto steering")
                self.mc.steering_active.clear()
        elif pgn_id == PGN_STEER_SETTINGS:
            # gainP(1,uint8_t)	highPWM(1,uint8_t)	lowPWM(1,uint8_t)	minPWM(1,uint8_t)	countsPerDeg(2)	steerOffset(2)	ackermanFix(1)
            payload = data[5:-1]
            unpacked_payload = {
                "gainP": struct.unpack("<B", payload[0:1])[0],
                "highPWM": struct.unpack("<B", payload[1:2])[0],
                "lowPWM": struct.unpack("<B", payload[2:3])[0],
                "minPWM": struct.unpack("<B", payload[3:4])[0],
                "countsPerDeg": struct.unpack("<H", payload[4:6])[0],
                # steerSettings.wasOffset |= (Serial.read() << 8);  //read was zero offset Lo
                "steerOffset": struct.unpack("<H", payload[6:8])[0],
                # convert ackermanFix to percentage
                "ackermanFix": struct.unpack("<B", payload[8:9])[0] / 100.0,
            }
            self.logger.info(
                "Received AutoSteer settings from AgIO containing { gainP %d, highPWM %d, lowPWM %d, minPWM %d, countsPerDeg %d, steerOffset %d, ackermanFix %.2f }",
                unpacked_payload["gainP"],
                unpacked_payload["highPWM"],
                unpacked_payload["lowPWM"],
                unpacked_payload["minPWM"],
                unpacked_payload["countsPerDeg"],
                unpacked_payload["steerOffset"],
                unpacked_payload["ackermanFix"],
            )

        elif pgn_id == PGN_STEER_CONFIG:
            payload = data[5:-1]
            unpacked_payload = {
                "InvertWAS": payload[0] & 0x01,
                "IsRelayActiveHigh": (payload[0] >> 1) & 0x01,
                "MotorDriveDirection": (payload[0] >> 2) & 0x01,
                "SingleInputWAS": (payload[0] >> 3) & 0x01,
                "CytronDriver": (payload[0] >> 4) & 0x01,
                "SteerSwitch": (payload[0] >> 5) & 0x01,
                "SteerButton": (payload[0] >> 6) & 0x01,
                "ShaftEncoder": (payload[0] >> 7) & 0x01,
                "PulseCountMax": payload[1],
                "IsDanfoss": (payload[2] >> 0) & 0x01,
                "PressureSensor": (payload[2] >> 1) & 0x01,
                "CurrentSensor": (payload[2] >> 2) & 0x01,
                "IsUseY_Axis": (payload[2] >> 3) & 0x01,
            }

        elif pgn_id == PGN_HELLO_REQUEST:
            self.logger.info("Received Hello request from AgIO")
            self.send_hello_reply_steering()
        elif pgn_id == PGN_SUBNET_SCAN_REQUEST:
            self.logger.info("Received Subnet Scan request from AgIO")
            self.send_subnet_scan_reply(src_ip, src_port)

    def send_subnet_scan_reply(self, src_ip: str, src_port: int) -> None:
        """Send "Subnet Scan Reply" PGN to AgIO"""
        self.logger.info("Sending Subnet Scan Reply to AgIO")
        # uint8_t scanReply[] = { 128, 129, 126, 203, 7,
        #               networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 126,
        #               src_ip[0], src_ip[1], src_ip[2], checksum };
        # TODO: figure out why they use 126 as the fixed ip of the Autosteer controller. It is the same as the PGN source identifier but should not be required here.
        data = bytearray([0x80, 0x81, SOURCE_AUTOSTEER, PGN_SUBNET_SCAN_REPLY, 0x07])

        # network address
        network_address = self.server.getsockname()[0].split(".")
        assert len(network_address) == 4, "Expected 4 octets in a IPv4 address"
        data.extend([int(x) for x in network_address])

        # source ip
        src_ip_octets = src_ip.split(".")
        assert len(src_ip_octets) == 4, "Expected 4 octets in a IPv4 address"
        data.extend([int(x) for x in src_ip_octets[:3]])

        # print a warning if the source ip is not on the same subnet as the configured AGIO_NETWORK_IP
        if src_ip_octets[:3] != AGIO_NETWORK_IP.split(".")[:3]:
            self.logger.warning(
                "Source IP %s is not on the same subnet as AGIO_NETWORK_IP %s. This utility may not work as expected.",
                src_ip,
                AGIO_NETWORK_IP,
            )

        # checksum
        data.append(self.calc_crc(data))

        try:
            self.client.sendto(bytes(data), (AGIO_NETWORK_IP, AGIO_RECEIVE_PORT))
        except Exception:
            self.logger.exception("Unhandled exception while sending Subnet Scan Reply to AgIO")

    def send_hello_reply_steering(self) -> None:
        """Send "Hello Reply Steering" PGN to AgIO"""
        self.logger.info("Sending Hello Reply from AutoSteering to AgIO")
        # uint8_t helloFromAutoSteer[] = { 128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
        data = bytearray([0x80, 0x81, SOURCE_AUTOSTEER, PGN_HELLO_REPLY_STEERING_1, 0x05])

        # two bytes of steering angle multiplied by 100
        wheel_angle = self.mc.current_angle_was()
        wheel_angle_int = int(wheel_angle * 100)
        data.extend(list(struct.pack("<h", wheel_angle_int)))

        # two bytes of counts, not used
        # TODO: figure out if we need to come up with some value here
        wheel_angle_adc_counts = 0
        data.extend(list(struct.pack("<h", wheel_angle_adc_counts)))

        # switch byte, not used
        switch_byte = 0
        data.append(switch_byte)

        # crc is fixed to 71 for whatever reason
        data.append(71)

        try:
            self.client.sendto(bytes(data), (AGIO_NETWORK_IP, AGIO_RECEIVE_PORT))
        except Exception:
            self.logger.exception("Unhandled exception while sending Hello Reply Steering to AgIO")

    def report_actual_steering_data(self) -> None:
        heading = roll = switch = 0

        while True:
            wheel_angle = self.mc.current_angle_was()
            pwm_display = abs(self.mc.motor.getVelocity())
            self.send_from_autosteer(wheel_angle, heading, roll, switch, int(pwm_display * 255))

            # report the current consumption of the motor
            current_amps = self.mc.current_sensor.getCurrent()
            if current_amps is not None:
                self.logger.info("Motor current consumption: %.2f A", current_amps)
                sensor_value = int(current_amps * 10)  # convert to 0.1 A interval with 0-100 range
                self.send_sensor_value_from_autosteer(sensor_value)

            time.sleep(1.0 / WAS_REPORTING_FREQUENCY)

    def send_sensor_value_from_autosteer(self, sensor_value: int) -> None:
        """Send sensor value from AutoSteer to AgIO"""
        self.logger.info(
            "Sending sensor value from AutoSteer to AgIO containing sensor_value %.2f",
            sensor_value,
        )

        # the data format is 8 payload bytes and the first payload byte is the sensor value
        data = bytearray([0x80, 0x81, SOURCE_AUTOSTEER, PGN_DATA_FROM_AUTOSTEER_2, 0x08])
        data.append(sensor_value)
        # the rest of the payload is not used, so we fill it with 0
        for _ in range(7):
            data.append(0)

        data.append(self.calc_crc(data))

        try:
            self.client.sendto(bytes(data), (AGIO_NETWORK_IP, AGIO_RECEIVE_PORT))
        except Exception:
            self.logger.exception("Unhandled exception while sending sensor value to AgIO")

    def send_from_autosteer(
        self, wheel_angle: float, heading: float, roll: float, steer_switch: int, pwm_display: int
    ) -> None:
        """Send "From AutoSteer" PGN to AgIO"""
        self.logger.info(
            "Sending data from AutoSteer to AgIO containing wheel_angle %.2f", wheel_angle
        )

        # the data format is
        data = bytearray([0x80, 0x81, SOURCE_AUTOSTEER, PGN_DATA_FROM_AUTOSTEER, 0x08])
        wheel_angle_int = int(wheel_angle * 100)
        data.extend(list(struct.pack("<h", wheel_angle_int)))
        heading_int = int(heading * 10)
        data.extend(list(struct.pack("<h", heading_int)))
        roll_int = int(roll * 10)
        data.extend(list(struct.pack("<h", roll_int)))

        data.append(steer_switch)
        data.append(pwm_display)

        data.append(self.calc_crc(data))

        try:
            self.client.sendto(bytes(data), (AGIO_NETWORK_IP, AGIO_RECEIVE_PORT))
        except Exception:
            self.logger.exception("Unhandled exception while sending AutoSteer data to AgIO")

    def calc_crc(self, data):
        """return the "crc" byte of data

        The crc is calculated by summing all bytes (excluding the first two) and taking the modulo 256 of the sum
        """
        crc = 0
        for byte in data[2:]:
            crc += byte
        crc %= 256
        return crc


if __name__ == "__main__":
    try:
        steering_config = SteeringControllerConfig()

        mc = SteeringController()
        mc.calibrate_center()

        agas = AgIOAutsteer(mc)
        agas.report_actual_steering_data()

    except KeyboardInterrupt as _:
        logging.info("Received a keyboard interrupt")
    except Exception:
        logging.exception("Unhandled Exeception occured")
    finally:
        try:
            agas.shutdown()
        except NameError:
            # maybe agas was not initialized yet
            pass
        mc.shutdown()
