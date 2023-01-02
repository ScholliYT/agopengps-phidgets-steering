import logging
import time
import socket
import struct
import threading

from agopengps_phidgets_steering.steering_controller import SteeringController


logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)-8s %(name)-15s %(message)s')

SOURCE_AGIO = 0x7f
PGN_AUTOSTEER_DATA = 0xfe

class AgIOAutsteer:

    def __init__(self, mc: SteeringController):
        self.logger = logging.getLogger(name="AgIOAutsteer")

        assert mc.steering_wheel_full_range > 0, "Looks like the steering wheel range is not calibrated"
        self.mc = mc

        # Client to send messages back to AgIO
        self.client=socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.client.settimeout(0)
        self.client.setblocking(0)

        # Server to receive messages from AgIO
        self.server=socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.server.bind(('', 8888))
        self.server.settimeout(0.1)

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
                    (data, address) = self.server.recvfrom(1024)
                    if data[0] == 0x80 and data[1] == 0x81:
                        self.decode_data(data)
            except socket.timeout as e:
                self.logger.error("Timeout error reading UDP data from AgIO", exc_info=e)
            except socket.error as e:
                self.logger.error("Error reading UDP data from AgIO", exc_info=e)
    
    def decode_data(self, data) -> None:
        data_source = data[2]
        pgn_id = data[3]
        payload_length = data[4]

        data_crc = data[-1]
        received_crc = self.calc_crc(data[:-1])

        if data_crc != received_crc:
            self.logger.warning("Received data with invalid crc")
            return

        # Autosteer data from AgIO
        if data_source == SOURCE_AGIO and pgn_id == PGN_AUTOSTEER_DATA:
            payload = data[5:-1]
            unpacked_payload = { 
                'Speed': struct.unpack('<H', payload[0:2])[0]/10.0,
                'AutosteerActive': payload[2],
                'SteerAngle': struct.unpack('<h', payload[3:5])[0]/100.0,
                'SectionControl': struct.unpack('<H', payload[6:8])[0],
            }
            self.logger.info("Received AutoSteer data from AgIO containing { Speed %.1f, Steer angle %.2f, Autosteer %s, Section control %d}.", \
                unpacked_payload['Speed'],
                unpacked_payload['SteerAngle'],
                "active" if unpacked_payload['AutosteerActive'] else "inactive",
                unpacked_payload['SectionControl'])
            mc.target_angle = float(unpacked_payload['SteerAngle'])
            
            if not self.mc.steering_active.is_set() and unpacked_payload['AutosteerActive']:
                self.logger.info("Activating motor for auto steering")
                self.mc.steering_active.set()
            elif self.mc.steering_active.is_set() and not unpacked_payload['AutosteerActive']:
                self.logger.info("Deactivating motor for auto steering")
                self.mc.steering_active.clear()

    def report_actual_steering_data(self) -> None:
        heading = roll = switch = 0

        while True:
            wheel_angle = mc.current_angle()
            pwm_display = abs(mc.motor.getVelocity())
            self.send_from_autosteer(wheel_angle, heading, roll, switch, int(pwm_display * 255))
            time.sleep(1/5)

    def send_from_autosteer(self, wheel_angle: float, heading: float, roll: float, steer_switch: int, pwm_display: int) -> None:
        """ Send "From AutoSteer" PGN to AgIO
        """
        self.logger.info("Sending data from AutoSteer to AgIO containing wheel_angle %.2f", wheel_angle)

        data = bytearray([0x80, 0x81, 0x7e, 0xfd, 0x08])
        wheel_angle_int = int(wheel_angle * 100)
        data.extend(list(struct.pack('<h', wheel_angle_int)))
        heading_int = int(heading * 10)
        data.extend(list(struct.pack('<h', heading_int)))
        roll_int = int(roll * 10)
        data.extend(list(struct.pack('<h', roll_int)))

        data.append(steer_switch)
        data.append(pwm_display)

        data.append(self.calc_crc(data))

        try:
            self.client.sendto(bytes(data), ('192.168.178.255',9999))
        except Exception:
            self.logger.exception("Unhandled exception while sending AutoSteer data to AgIO")

    def calc_crc(self, data):
        """ return the "crc" byte of data
        """
        crc = 0
        for byte in data[2:]:
            crc += byte
        crc %= 256
        return crc


if __name__ == '__main__':
    try:
        mc = SteeringController()
        mc.calibrate_center()

        agas = AgIOAutsteer(mc)
        agas.report_actual_steering_data()

        

    except KeyboardInterrupt as _:
        logging.info("Received a keyboard interrupt")
    except Exception:
        logging.exception("Unhandled Exeception occured")
    finally:
        agas.shutdown()
        mc.shutdown()
