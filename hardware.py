import smbus2
import time
import requests
import numpy as np
import logging

# Configure logging
logging.basicConfig(filename='sensett.log', level=logging.INFO)
logger = logging.getLogger(__name__)

# Supported sensor classes and their metadata
SUPPORTED_SENSORS = {
    "SHT31": {
        "class": "SHT31Sensor",
        "data_types": ["temp", "hum"],
        "data_names": ["Temperature", "Humidity"],
        "data_units": ["°C", "%"],
        "notes": [
            "SHT31 is an i2c device. Ensure it is wired to your Pi correctly. Docs: https://learn.adafruit.com/adafruit-sht31-d-temperature-and-humidity-sensor-breakout/python-circuitpython",
            "SHT31 has a fixed i2c address (0x44). Just like any i2c device, you may wire multiple devices to the same i2c bus, so long as no device shares an address. Since all SHT31s share the 0x44 address, you will be unable to wire multiple SHT31s to the same bus. If you wish to run several SHT31s, you will need to enable multiple i2c buses and wire each device to a different bus, or otherwise implement a hardware i2c multiplexer",
            "Ensure your i2c buses have been enabled on the Pi with either the \"dtoverlay=i2c-gpio\" or \"dtparam=i2c_arm=on\" within the /boot/config.txt file. Docs: https://www.raspberrypi.com/documentation/computers/config_txt.html#common-hardware-configuration-options"
        ]
    },
    "SGP40": {
        "class": "SGP40Sensor",
        "data_types": ["aqi","index"],
        "data_names": ["Raw","Air Quality"],
        "data_units": [""],
        "notes": [
            "SGP40 is an i2c device. Ensure it is wired to your Pi correctly. Docs: https://learn.adafruit.com/adafruit-sgp40/python-circuitpython",
            "SGP40 has a fixed i2c address (0x59). Just like any i2c device, you may wire multiple devices to the same i2c bus, so long as no device shares an address. Since all SGP40s share the 0x59 address, you will be unable to wire multiple SGP40s to the same bus. If you wish to run several SGP40s, you will need to enable multiple i2c buses and wire each device to a different bus, or otherwise implement a hardware i2c multiplexer",
            "Ensure your i2c buses have been enabled on the Pi with either the \"dtoverlay=i2c-gpio\" or \"dtparam=i2c_arm=on\" within the /boot/config.txt file. Docs: https://www.raspberrypi.com/documentation/computers/config_txt.html#common-hardware-configuration-options"
        ]
    }
}


class SHT31Sensor:
    def __init__(self, config):
        """
        Initialize the SHT31 sensor.
        
        :param config: The sensor configuration dictionary
        """
        self.i2c_bus = config['i2c_bus']
        self.name = config['name']
        self.address = 0x44
        self.temp = 0
        self.hum = 0

    def read_data(self):
        """
        Read temperature and humidity data from the SHT31 sensor.
        
        :return: A tuple containing temperature and humidity
        """
        bus = smbus2.SMBus(self.i2c_bus)
        try:
            bus.write_i2c_block_data(self.address, 0x2C, [0x06])
            time.sleep(0.5)
            data = bus.read_i2c_block_data(self.address, 0x00, 6)
            temperature = -45 + (175 * ((data[0] * 256 + data[1]) / 65535.0))
            humidity = 100 * ((data[3] * 256 + data[4]) / 65535.0)
            return temperature, humidity
        except Exception as e:
            logger.error(f"Error reading from SHT31 on bus {self.i2c_bus}: {e}")
            return None, None
        finally:
            bus.close()

class SGP40Sensor:
    def __init__(self, config):
        """
        Initialize the SGP40 sensor.
        
        :param config: The sensor configuration dictionary
        """
        self.i2c_bus = config['i2c_bus']
        self.name = config['name']
        self.address = 0x59
        self.aqi = 0
        self.index = 0

    def read_data(self, temperature, humidity):
        """
        Read air quality data from the SGP40 sensor, compensating with temperature and humidity if provided.
        
        :param temperature: The temperature for compensation
        :param humidity: The humidity for compensation
        :return: The air quality value
        """
        bus = smbus2.SMBus(self.i2c_bus)
        try:
            if temperature is not None and humidity is not None:
                compensated_read_cmd = [0x26, 0x0F]
                hum_ticks = int((humidity * 65535) / 100 + 0.5) & 0xFFFF
                humidity_ticks = [(hum_ticks >> 8) & 0xFF, hum_ticks & 0xFF]
                humidity_ticks.append(self.generate_crc(humidity_ticks))
                tem_ticks = int(((temperature + 45) * 65535) / 175) & 0xFFFF
                temp_ticks = [(tem_ticks >> 8) & 0xFF, tem_ticks & 0xFF]
                temp_ticks.append(self.generate_crc(temp_ticks))
                command = compensated_read_cmd + humidity_ticks + temp_ticks
                bus.write_i2c_block_data(self.address, command[0], command[1:])
                time.sleep(0.25)  # Wait for sensor processing
                response = bus.read_i2c_block_data(self.address, 0x00, 6)
                raw_value = (response[0] << 8) | response[1]
                return raw_value
            else:
                return None
        except Exception as e:
            logger.error(f"Error reading from SGP40 on bus {self.i2c_bus}: {e}")
            return None
        finally:
            bus.close()

    def aqi(raw_data):
        try:
            if raw_data > 30999:
                return (100/-3000)*(raw_data-34000)
            elif raw_data > 30499:
                return ((99/-499)*(raw_data-30999))+101
            elif raw_data > 29999:
                return ((99/-499)*(raw_data-30499))+201
            elif raw_data > 29499:
                return ((99/-499)*(raw_data-29999))+301
            else:
                return ((99/-29499)*(raw_data-29499))+401
        except Exception as e:
            logger.error(f"Error indexing SGP40: {e}")
            return None

    @staticmethod
    def generate_crc(crc_buffer):
        """
        Generate CRC for data validation.
        
        :param crc_buffer: The buffer to generate CRC for
        :return: The CRC value
        """
        crc = 0xFF
        for byte in crc_buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc & 0xFF  # Returns only bottom 8 bits

FAN_CONTROL_URL = "http://localhost/printer/gcode/script"
class FanConfig:
    def __init__(self, name, trigger_sensor, trigger_value, trigger_on_above, kp=0.5, ki=0.1, kd=0.05):
        """
        Initialize the fan configuration with PID control parameters.
        """
        self.name = name
        self.control_url = FAN_CONTROL_URL
        self.trigger_sensor = trigger_sensor
        self.trigger_value = trigger_value
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.current_speed = 0
        self.integral = 0
        self.previous_error = 0
        self.last_update_time = time.time()

        # Adaptive tuning variables
        self.performance_history = []
        self.tuning_interval = 20  # Tune every 60 seconds
        self.last_tuning_time = time.time()

    def check_and_update_fan_speed(self, sensor_value):
        """
        Adjust the fan speed based on PID control to maintain the target value.
        """
        current_time = time.time()
        delta_time = current_time - self.last_update_time
        error = self.trigger_value - sensor_value

        # Reset integral term if the error changes sign (threshold crossing)
        if error * self.previous_error < 0:
            self.integral = 0
            logger.debug(f"Integral term reset for fan '{self.name}' due to threshold crossing.")

        # PID terms
        proportional = self.kp * error
        self.integral += error * delta_time
        integral = self.ki * self.integral
        derivative = self.kd * (error - self.previous_error) / delta_time if delta_time > 0 else 0

        # Calculate PID output
        output = (proportional + integral + derivative) / 1000

        # Clamp output to [0.0, 1.0]
        speed = max(0.0, min(1.0, output))

        # Log PID terms and speed
        logger.info(
            f"Fan '{self.name}': PID Output: {output:.2f}, Error={error:.2f}, P={proportional:.2f}, "
            f"I={integral:.2f}, D={derivative:.2f}, Speed={speed:.2f}"
        )

        # Update fan speed if it changes significantly
        if abs(self.current_speed - speed) > 0.01:  # Threshold for update
            self.set_fan_speed(speed)

        # Track performance for adaptive tuning
        self.performance_history.append(abs(error))
        if len(self.performance_history) > 50:  # Limit history to the last 100 entries
            self.performance_history.pop(0)

        # Perform adaptive tuning periodically
        if current_time - self.last_tuning_time >= self.tuning_interval:
            self.tune_pid_parameters()
            self.last_tuning_time = current_time

        # Update state for the next iteration
        self.previous_error = error
        self.last_update_time = current_time

    def tune_pid_parameters(self):
        """
        Adaptively tune PID parameters based on recent performance.
        """
        if not self.performance_history:
            return

        # Calculate performance metrics
        avg_error = np.mean(self.performance_history)
        max_error = max(self.performance_history)

        # Adaptive tuning logic
        if max_error > 10:  # Example threshold for large oscillations
            logger.info(f"High oscillations detected for fan '{self.name}'. Adjusting PID parameters.")
            self.kp = max(0.1, self.kp - 0.1)  # Reduce Kp to minimize overshooting
            self.kd = min(0.5, self.kd + 0.05)  # Increase Kd for better damping
        elif avg_error > 5:  # Example threshold for slow response
            logger.info(f"High average error detected for fan '{self.name}'. Increasing Kp.")
            self.kp = min(1.0, self.kp + 0.1)  # Increase Kp to improve responsiveness
        elif avg_error < 1:  # Example threshold for steady-state error
            logger.info(f"Low steady-state error detected for fan '{self.name}'. Increasing Ki.")
            self.ki = min(0.5, self.ki + 0.05)  # Increase Ki to correct steady-state error

        # Log the updated PID coefficients
        logger.info(f"Updated PID parameters for fan '{self.name}': Kp={self.kp:.2f}, Ki={self.ki:.2f}, Kd={self.kd:.2f}")

    def set_fan_speed(self, speed):
        """
        Set the fan speed via HTTP request.
        """
        payload = {"script": f"SET_FAN_SPEED FAN={self.name} SPEED={speed}"}
        headers = {"Content-Type": "application/json"}
        response = requests.post(self.control_url, json=payload, headers=headers)
        if response.status_code == 200:
            self.current_speed = speed
            logger.info(f"Fan '{self.name}' speed set to {speed:.2f}")
        else:
            logger.error(f"Failed to set fan speed: {response.status_code}")

class MQTTSensor:
    def __init__(self, name, hardware_sensors):
        """
        Initialize the MQTT sensor.
        
        :param name: The name of the MQTT sensor
        :param hardware_sensors: The hardware sensors associated with this MQTT sensor
        """
        self.name = name.lower()
        self.mqtt_topic = f"sensor/{self.name}"
        self.hardware_sensors = hardware_sensors

def create_sensor(config):
    """
    Factory function to create sensor instances based on the sensor configuration.
    
    :param config: The sensor configuration dictionary
    :return: An instance of the sensor class
    """
    sensor_class = globals()[SUPPORTED_SENSORS[config['type']]["class"]]
    return sensor_class(config)
