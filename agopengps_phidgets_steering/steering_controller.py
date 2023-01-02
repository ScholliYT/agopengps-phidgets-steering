import time
import logging
import threading

from Phidget22.ErrorEventCode import ErrorEventCode
from Phidget22.Devices.DCMotor import DCMotor
from Phidget22.Devices.VoltageInput import VoltageInput
from Phidget22.Devices.CurrentInput import CurrentInput
from Phidget22.Devices.Encoder import Encoder

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)-8s %(name)-15s %(message)s')

OVERCURRENT_LIMIT = 0.4 # limit max current in ampere the motor may draw
MAX_STEERING_ANGLE = 45.0 # max angle you can turn the steering wheels (from -MAX_STEERING_ANGLE to +MAX_STEERING_ANGLE)
INVERT_MOTOR_DIR = True # Set to True if you want to invert the rotation of the motor (if you mount the motor from the bottom)
CONTORL_LOOP_FREQUENCY = 50 # Configure the loop frequency of the PI controller in Hz

# Parameters for PI controller
kp = 0.01
ki = 0.001

class SteeringController:
    """ Control logic for the steering wheel by interfacing the Phidgets Motor Controller
    """

    def __init__(self):
        self.logger = logging.getLogger(name="MotorController")

        # Create your Phidget channels
        self.motor = DCMotor()
        self.supply_voltage_sensor = VoltageInput()
        self.current_sensor = CurrentInput()
        self.encoder = Encoder()

        # Set addressing parameters to specify which channel to open (if any)
        self.supply_voltage_sensor.setChannel(2)

        # Assign any event handlers you need before calling open so that no events are missed.
        self.motor.setOnAttachHandler(self.motor_attached)
        self.motor.setOnDetachHandler(self.motor_detached)
        self.supply_voltage_sensor.setOnVoltageChangeHandler(self.on_voltage_change)
        self.current_sensor.setOnCurrentChangeHandler(self.on_current_change)
        self.encoder.setOnAttachHandler(self.encoder_attach)
        self.encoder.setOnPositionChangeHandler(self.check_encoder_position)

        # Open your Phidgets and wait for attachment
        self.motor.openWaitForAttachment(2000)
        self.supply_voltage_sensor.openWaitForAttachment(2000)
        self.current_sensor.openWaitForAttachment(2000)
        self.encoder.openWaitForAttachment(2000)

        # Steering control
        self.target_angle: float = 0.0
        self.running = threading.Event()
        self.running.set()
        self.steering_active = threading.Event()
        self.control_loop_thread = threading.Thread(target=self.control_loop)
        self.control_loop_thread.start()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self.logger.info("Stopping control loop")
        self.steering_active.clear()
        self.running.clear()
        self.control_loop_thread.join(1)

        if self.motor.getIsOpen():
            self.logger.info("Stopping motor")
            self.motor.setTargetVelocity(0)
            self.motor.setTargetBrakingStrength(0)

        # wait some time for control loops to terminate
        time.sleep(0.2)

        # ensure the motor is stopped
        if self.motor.getIsOpen():
            self.motor.setTargetVelocity(0)
            self.motor.setTargetBrakingStrength(0)

        self.logger.info("Closing sensor objects")
        if self.motor.getIsOpen(): self.motor.close()
        if self.supply_voltage_sensor.getIsOpen(): self.supply_voltage_sensor.close()
        if self.current_sensor.getIsOpen(): self.current_sensor.close()
        if self.encoder.getIsOpen(): self.encoder.close()

    def motor_attached(self, _):
        self.logger.info("Motor attached!")
        self.motor.setAcceleration(5.0)
        self.motor.setDataRate(50) # set up to 125.0 Hz
        self.motor.setOnErrorHandler(self.error_handler)

    def encoder_attach(self, _):
        self.encoder.setDataRate(100) # set up to 125.0 Hz
        self.encoder.setPositionChangeTrigger(10) # TODO: set to resonable value (HKT22 has 300 counts per rotation)
    
    def check_encoder_position(self, _, positionChange, timeChange, indexTriggered):
        # only check if the steering system is active
        if not self.steering_active.is_set():
            return

        # ensure that the current steering angle is in a acceptable range
        # acceptable is -1.5*MAX_STEERING_ANGLE to 1.5*MAX_STEERING_ANGLE
        # if thats not the case something went wrong and we terminate
        min_deg = -1.5 * MAX_STEERING_ANGLE
        max_deg = +1.5 * MAX_STEERING_ANGLE
        current_angle = self.current_angle()

        if min_deg > current_angle or \
           max_deg < current_angle:
            self.logger.error("The steering wheel is at %.2f°, which is outside of the acceptable range of %.2f° to %.2f°. Terminating now.", current_angle, min_deg, max_deg)
            self.shutdown()

    
    def motor_detached(self, _):
        self.logger.warning("Motor detached!")
        self.shutdown()
    
    def error_handler(self, _, code, description):
        self.logger.error("Error with Code '%s' occured. Description: %s", ErrorEventCode.getName(code), str(description))

    def on_voltage_change(self, _, voltage):
        self.logger.debug("Voltage: " + str(voltage))

    def on_current_change(self, _, current: float):
        self.logger.debug("Current: " + str(current))

        if current > OVERCURRENT_LIMIT:
            self.logger.warning("The motor has consumed %.2f amps, exceeding the allowed %.2f amps.", current, OVERCURRENT_LIMIT)


    def current_angle(self) -> float:
        """
        Returns the current steering wheel angle in degrees.
        It uses the motor's encoder value to calculate the current angle.
        """
        angle = MAX_STEERING_ANGLE * self.encoder.getPosition() / (self.steering_wheel_full_range//2)

        return angle


    def delta_angle(self, target_angle: float) -> float:
        return self.current_angle() - target_angle

    def calibrate_center(self):
        """Perform a user guided calibration of the steering wheel.
        In order to know the number of Encoder ticks that correspond to the full range of the steering wheel
        we need to capture this manually.


        0                                   should be the left endpoint of the steering wheel
        self.steering_wheel_full_range/2    should be the center of the steering wheel
        self.steering_wheel_full_range      should be the right endpoint of the steering wheel
        """

        self.motor.setTargetVelocity(0.0)
        self.motor.setTargetBrakingStrength(0.0)
        input("Turn steering wheel to the left endpoint and press Enter\n")

        self.encoder.setPosition(0)
        input("Turn steering wheel to the right endpoint and press Enter\n")

        self.steering_wheel_full_range: int = self.encoder.getPosition()
        if INVERT_MOTOR_DIR:
            # we expect counter-clockwise rotation of the motor
            assert self.steering_wheel_full_range < 0, "expected full steering range to be negative"
        else:
            # we expect clockwise rotation of the motor
            assert self.steering_wheel_full_range > 0, "expected full steering range to be positive"
        self.logger.info("Total steering wheel range %d", self.steering_wheel_full_range)

        input("Press Enter to center steering wheel\n")
        self.encoder.setPosition(self.steering_wheel_full_range//2)

        # Center steering wheel
        start_time = time.time()
        self.steering_active.set()
        
        # wait till we are settled at almost the center or timeout of 5 seconds
        while self.delta_angle(0) > 2 or abs(self.motor.getVelocity()) > 0.1 or \
              time.time() - start_time < 5:
            continue

        self.steering_active.clear()
        self.motor.setTargetVelocity(0)

        self.logger.info("Motor centered with final error of %.2f", self.delta_angle(0))

    def control_loop(self) -> None:
        """ Control the motors velocity using a PI controller
        Given the target angle (self.target_angle) the controller 
        constantly updates the motor's velocity to minimize the error aka. delta_angle.
        """
        
        error = last_error = error_sum = 0.0
        while self.running.is_set():
            if not self.steering_active.is_set():
                time.sleep(0.1)
                self.motor.setTargetVelocity(0)
                self.motor.setTargetBrakingStrength(0)
                continue

            start_time = time.time()

            # reset integral error on sign flip
            if error * last_error > 0:
                error_sum += error
            else:
                error_sum = 0

            velocity = (-1) * (kp * error + ki * error_sum)

            if INVERT_MOTOR_DIR:
                velocity *= -1

            # limit to max velocity (i.e. -1 to +1)
            velocity = min(velocity, self.motor.getMaxVelocity())
            velocity = max(velocity, -self.motor.getMaxVelocity())

            self.logger.info("Steering from %.2f° to %.2f° with Motor velocity of %.3f and current error %.2f° | error sum %.2f", self.current_angle(), self.target_angle, velocity, error, error_sum)
            self.motor.setTargetVelocity(velocity)

            last_error = error
            error = self.delta_angle(self.target_angle)

            # limit frequency
            exec_time = time.time() - start_time
            time.sleep(max(0, 1/CONTORL_LOOP_FREQUENCY - exec_time))
            self.logger.debug("Control loop execution time %f", exec_time)


    def start_manual_input_steering(self):
        self.target_angle = 0.0
        self.steering_active.set()

        try:
            while self.running.isSet():
                cmd = input("Enter the target steering angle or 'stop' to terminate")

                if cmd.lstrip("+-").isdecimal():
                    target_angle = float(cmd)

                    if target_angle > MAX_STEERING_ANGLE:
                        self.target_angle = MAX_STEERING_ANGLE
                        self.logger.warning("The requested steering angle %.2f is more than the maximal steering angle of %.2f", target_angle, MAX_STEERING_ANGLE)
                    elif target_angle < -MAX_STEERING_ANGLE:
                        self.target_angle = -MAX_STEERING_ANGLE
                        self.logger.warning("The requested steering angle %.2f is less than the minimal steering angle of %.2f", target_angle, -MAX_STEERING_ANGLE)
                    else:
                        self.target_angle = target_angle
                elif cmd.lower() == 'stop':
                    break
        except KeyboardInterrupt as _:
            self.logger.info("Stopping steering after receiving a keyboard interrupt")
        self.logger.info("Stopping steering")
        self.steering_active.clear()
        self.running.clear()

if __name__ == '__main__':
    try:
        mc = SteeringController()
        mc.calibrate_center()
        time.sleep(2)
        mc.start_manual_input_steering()
    except Exception:
        logging.exception("Unhandled Exeception occured")
    finally:
        logging.info("Shutting down")
        try: 
            mc.shutdown()
        except Exception:
            logging.exception("Unhandled Exeception occured in final shutdown")
