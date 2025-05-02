from dataclasses import dataclass
import dataclasses
import os
import time
import logging
import threading
import yaml


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(levelname)-8s %(name)-15s %(message)s"
)


@dataclass
class SteeringControllerConfig:
    """Configuration Parameters for the SteeringController"""

    left_endpoint_voltage: float = 0.0
    right_endpoint_voltage: float = 0.0

    # PI-Controller parameters
    kp: float = 0.01
    ki: float = 0.001
    # Other motor controller parameters

    # Motor parameters
    calibration_steering_angle_deg: float = 35.0
    invert_motor_dir: bool = True
    invert_was_dir: bool = True
    overcurrent_limit_amps: float = 2.4  # limit max current in ampere the motor may draw

    # Control loop parameters
    control_loop_frequency_hz: float = (
        50.0  # Configure the loop frequency of the PI controller in Hz
    )

    def to_yaml(self) -> str:
        """Convert the configuration to a YAML string"""
        return yaml.dump(dataclasses.asdict(self), default_flow_style=False)

    @classmethod
    def from_yaml(cls, yaml_str: str) -> "SteeringControllerConfig":
        """Load the configuration from a YAML string"""
        config_dict = yaml.safe_load(yaml_str)
        return cls(**config_dict)


class SteeringControllerConfigFromFile(SteeringControllerConfig):
    """Configuration Parameters for the SteeringController to be loaded from a YAML file with live bi-directional updates"""

    def __init__(self, absolute_file_path: str, **kwargs):
        super().__init__(**kwargs)

        self.absolute_file_path = absolute_file_path

        self._watcher_thread: threading.Thread = None
        self.is_watching = threading.Event()
        self.listen_for_changes_lock = threading.Lock()
        self.last_modified_time = None

        # Check if the file exists, if not create it with default values
        if not os.path.exists(self.absolute_file_path):
            logging.info("Config file does not exist, creating it with default values...")
            with open(self.absolute_file_path, "w") as f:
                f.write(self.to_yaml())
        else:
            logging.info("Config file exists, loading it...")
            # Load the existing configuration
            self._load_config()

        self._watch_file()

    def _load_config(self):
        """Load the configuration from the YAML file"""
        try:
            with open(self.absolute_file_path, "r") as f:
                config = SteeringControllerConfig.from_yaml(f.read())
                self.__dict__.update(config.__dict__)
        except Exception as e:
            logging.error("Failed to load config file: %s", e)
            raise

    def _watch_file(self):
        """Watch the file for changes and reload the configuration if it changes"""

        self.is_watching.set()

        def watch_file():
            self.last_modified_time = os.path.getmtime(self.absolute_file_path)
            while self.is_watching.is_set():
                time.sleep(1)
                if self.listen_for_changes_lock.acquire(timeout=1):
                    current_modified_time = os.path.getmtime(self.absolute_file_path)
                    if current_modified_time != self.last_modified_time:
                        logging.info("Config file changed, reloading...")
                        try:
                            self._load_config()
                        except Exception as e:
                            logging.error("Failed to reload config file: %s", e)
                        finally:
                            self.last_modified_time = current_modified_time
                    self.listen_for_changes_lock.release()

        self._watcher_thread = threading.Thread(target=watch_file, daemon=True).start()

    def __del__(self):
        """Stop the watcher thread"""
        if hasattr(self, "_watcher_thread") and self._watcher_thread is not None:
            self.is_watching.clear()
            self._watcher_thread.join()
            logging.info("Stopped watching config file.")

    def __setattr__(self, name, value):
        # intercept attribute setting to update the YAML file
        super().__setattr__(name, value)

        # Check if watching is running by flag but first check if the flag is actucally available (has attr)
        if hasattr(self, "is_watching") and self.is_watching.is_set():
            config_attributes = [
                f.name
                for f in dataclasses.fields(SteeringControllerConfig)
                if f.init and f.name != "absolute_file_path"
            ]
            if name in config_attributes:
                if self.listen_for_changes_lock.acquire(timeout=3):
                    with open(self.absolute_file_path, "w") as f:
                        f.write(self.to_yaml())
                    self.last_modified_time = os.path.getmtime(self.absolute_file_path)
                    self.listen_for_changes_lock.release()
                else:
                    logging.warning(
                        "Failed to acquire lock to update config file. Changes will not be saved."
                    )
