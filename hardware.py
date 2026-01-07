import smbus2
import time
import requests
import numpy as np
import logging

# Configure logging
logging.basicConfig(filename='~/printer_data/logs/sensett.log', level=logging.INFO)
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
        "data_types": ["aqi"],
        "data_names": ["Air Quality"],
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
    def __init__(self, name, trigger_sensor, trigger_value, trigger_on_above):
        """
        Initialize the fan configuration.
        
        :param name: The name of the fan
        :param trigger_sensor: The sensor that triggers the fan
        :param trigger_value: The value to turn the fan on
        :param trigger_on_above: Boolean indicating if the fan should turn on when the value is above the trigger value
        """
        self.name = name
        self.control_url = FAN_CONTROL_URL
        self.trigger_sensor = trigger_sensor
        self.trigger_value = trigger_value
        self.trigger_on_above = trigger_on_above
        self.current_speed = 0

    def check_and_update_fan_speed(self, sensor_value):
        """
        Check and update the fan speed based on sensor value.
        
        :param sensor_value: The current sensor value
        """
        if (self.trigger_on_above and sensor_value >= self.trigger_value) or (not self.trigger_on_above and sensor_value <= self.trigger_value):
            if self.current_speed != 1.0:
                self.set_fan_speed(1.0)
        else:
            if self.current_speed != 0:
                self.set_fan_speed(0)

    def set_fan_speed(self, speed):
        """
        Set the fan speed.
        
        :param speed: The speed to set the fan to
        """
        payload = {"script": f"SET_FAN_SPEED FAN={self.name} SPEED={speed}"}
        headers = {"Content-Type": "application/json"}
        response = requests.post(self.control_url, json=payload, headers=headers)
        if response.status_code == 200:
            self.current_speed = speed
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
