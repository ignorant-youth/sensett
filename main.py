import os
import time
import json
import numpy as np
import logging
from threading import Thread
import paho.mqtt.client as mqtt
from hardware import create_sensor, FanConfig, MQTTSensor, SUPPORTED_SENSORS

# Logging configuration
logging.basicConfig(filename='~/printer_data/logs/sensett.log', level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Configuration file path
CONFIG_FILE = 'sensett.conf'

def load_config():
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(script_dir, CONFIG_FILE)
        with open(config_path, 'r') as file:
            raw_content = file.read()
            logger.debug(f"Raw config file content: {raw_content}")
            config = json.loads(raw_content)
            return config
    except Exception as e:
        logger.error(f"Error loading config file: {e}")
        return {}

# Load configuration
config = load_config()

# Initialize hardware sensors
hardware_sensors = [create_sensor(sensor_conf) for sensor_conf in config.get('HARDWARE_SENSORS', [])]

# Initialize fans
fan = FanConfig(**config['FAN']) if 'FAN' in config and config['FAN'] else None

# Initialize MQTT sensors
mqtt_sensors = [MQTTSensor(sensor_conf['name'], sensor_conf['hardware_sensors']) for sensor_conf in config.get('MQTT_SENSORS', [])]

# MQTT setup
MQTT_BROKER = config.get('MQTT_BROKER', 'localhost')
MQTT_PORT = config.get('MQTT_PORT', 1883)

def setup_mqtt_client(broker, port):
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    client.connect(broker, port, 60)
    return client

mqtt_client = setup_mqtt_client(MQTT_BROKER, MQTT_PORT)

# Process data
def process_data():
    sample_count = 10
    temp_humidity_data = {}
    air_quality_data = {}
    
    for sensor in hardware_sensors:
        sensor_type = type(sensor).__name__
        if sensor_type == 'SHT31Sensor':
            temp_humidity_data[sensor.name] = ([], [])
        elif sensor_type == 'SGP40Sensor':
            air_quality_data[sensor.name] = []

    while True:
        try:
            for sensor in hardware_sensors:
                sensor_type = type(sensor).__name__
                if sensor_type == 'SHT31Sensor':
                    temp, humidity = sensor.read_data()
                    if temp is not None and humidity is not None:
                        temp_humidity_data[sensor.name][0].append(temp)
                        temp_humidity_data[sensor.name][1].append(humidity)
                        if len(temp_humidity_data[sensor.name][0]) > sample_count:
                            temp_humidity_data[sensor.name][0].pop(0)
                            temp_humidity_data[sensor.name][1].pop(0)
                        sensor.temp = round(np.mean(temp_humidity_data[sensor.name][0]), 2)
                        sensor.hum = round(np.mean(temp_humidity_data[sensor.name][1]), 2)
                elif sensor_type == 'SGP40Sensor':
                    # Find related SHT31Sensor if exists
                    related_sht31_sensor = None
                    for mqtt_sensor in mqtt_sensors:
                        if sensor.name in mqtt_sensor.hardware_sensors:
                            for hs in mqtt_sensor.hardware_sensors:
                                if hs != sensor.name:
                                    related_sht31_sensor = next((s for s in hardware_sensors if s.name == hs), None)
                                    break
                    
                    temp = related_sht31_sensor.temp if related_sht31_sensor else None
                    humidity = related_sht31_sensor.hum if related_sht31_sensor else None
                    air_quality = sensor.read_data(temp, humidity)
                    if air_quality is not None:
                        air_quality_data[sensor.name].append(air_quality)
                        if len(air_quality_data[sensor.name]) > sample_count:
                            air_quality_data[sensor.name].pop(0)
                        sensor.aqi = round(np.mean(air_quality_data[sensor.name]), 0)
            
            # Update fan speed if configured
            if fan:
                trigger_sensor = next((s for s in hardware_sensors if s.name == fan.trigger_sensor), None)
                if trigger_sensor:
                    sensor_type = type(trigger_sensor).__name__
                    trigger_value = None
                    if sensor_type == 'SGP40Sensor':
                        trigger_value = trigger_sensor.aqi
                    elif sensor_type == 'SHT31Sensor':
                        trigger_value = trigger_sensor.temp
                    if trigger_value is not None:
                        fan.check_and_update_fan_speed(trigger_value)

        except Exception as e:
            logger.error(f"Exception in process_data loop: {e}")

        time.sleep(1)

# Publish sensor data
def publish_sensor_data():
    time.sleep(60)  # Wait for initial data to be processed
    while True:
        try:
            for mqtt_sensor in mqtt_sensors:
                sensor_data_filtered = {}
                for sensor_name in mqtt_sensor.hardware_sensors:
                    sensor = next((s for s in hardware_sensors if s.name == sensor_name), None)
                    if sensor:
                        sensor_type = type(sensor).__name__.replace("Sensor", "")
                        for data_type in SUPPORTED_SENSORS[sensor_type]["data_types"]:
                            sensor_data_filtered[data_type] = getattr(sensor, data_type, None)
                mqtt_client.publish(mqtt_sensor.mqtt_topic, json.dumps(sensor_data_filtered))
            time.sleep(1)
        except Exception as e:
            logger.error(f"Exception in publish_sensor_data loop: {e}")
        time.sleep(1)

# Start data processing in a separate thread
data_thread = Thread(target=process_data)
data_thread.start()

# Start MQTT publishing in a separate thread
publish_thread = Thread(target=publish_sensor_data)
publish_thread.start()

mqtt_client.loop_forever()
