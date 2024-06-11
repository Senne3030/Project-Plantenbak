import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient

# MQTT settings
MQTT_BROKER = "senne3030.local" #Your raspberry pi address or <HostName>.local
MQTT_PORT = 1883
MQTT_TOPIC_TEMPERATURE = "home/plantenbak/temperature"
MQTT_TOPIC_HUMIDITY = "home/plantenbak/humidity"
MQTT_TOPIC_SOIL_TEMPERATURE = "home/plantenbak/Soiltemperature"
MQTT_TOPIC_SOIL_MOISTURE = "home/plantenbak/soilmoisture"
MQTT_TOPIC_LED = "home/plantenbak/led"
MQTT_TOPIC_WATER_PUMP = "home/plantenbak/waterPump"
MQTT_TOPIC_WATER_SENSOR = "home/plantenbak/watercontainer"
MQTT_TOPIC_FAN = "home/plantenbak/fan"
MQTT_USERNAME = "YOUR_MQTT_USERNAME"
MQTT_PASSWORD = "YOUR_MQTT_PASSWORD"

# InfluxDB settings
INFLUXDB_ADDRESS = "senne3030.local"
INFLUXDB_PORT = 8086
INFLUXDB_DATABASE = "YOUR_INFLUX_DATABASE"
INFLUXDB_USER = "YOUR_INFLUX_USERNAME"
INFLUXDB_PASSWORD = "YOUR_INFLUX_PASSWORD"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!") #If there are no errors subscribe to those topics
        client.subscribe(MQTT_TOPIC_TEMPERATURE)
        client.subscribe(MQTT_TOPIC_HUMIDITY)
        client.subscribe(MQTT_TOPIC_SOIL_TEMPERATURE)
        client.subscribe(MQTT_TOPIC_SOIL_MOISTURE)
        client.subscribe(MQTT_TOPIC_LED)
        client.subscribe(MQTT_TOPIC_WATER_PUMP)
        client.subscribe(MQTT_TOPIC_WATER_SENSOR)
        client.subscribe(MQTT_TOPIC_FAN)
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg): #If it has a certain topic save it to that certain measurement. This is true for all of those values.
    if msg.topic == MQTT_TOPIC_TEMPERATURE:
        temperature = float(msg.payload.decode())
        json_body = [
            {
                "measurement": "temperature",
                "fields": {
                    "value": temperature
                }
            }
        ]
        influxdb_client.write_points(json_body)
        print(f"Temperature data written to InfluxDB: {temperature}")
    elif msg.topic == MQTT_TOPIC_HUMIDITY:
        humidity = float(msg.payload.decode())
        json_body = [
            {
                "measurement": "humidity",
                "fields": {
                    "value": humidity
                }
            }
        ]
        influxdb_client.write_points(json_body)
        print(f"Humidity data written to InfluxDB: {humidity}")
    elif msg.topic == MQTT_TOPIC_SOIL_TEMPERATURE:
        soil_temperature = float(msg.payload.decode())
        json_body = [
            {
                "measurement": "soil_temperature",
                "fields": {
                    "value": soil_temperature
                }
            }
        ]
        influxdb_client.write_points(json_body)
        print(f"Soil temperature data written to InfluxDB: {soil_temperature}")

    elif msg.topic == MQTT_TOPIC_SOIL_MOISTURE:
        soil_moisture = float(msg.payload.decode())
        json_body = [
            {
                "measurement": "soil_moisture",
                "fields": {
                    "value": soil_moisture
                }
            }
        ]
        influxdb_client.write_points(json_body)
        print(f"Soil moisture data written to InfluxDB: {soil_moisture}")
    elif msg.topic == MQTT_TOPIC_LED:
        led_status = int(msg.payload.decode())
        json_body = [
            {
                "measurement": "led_status",
                "fields": {
                    "value": led_status
                }
            }
        ]
        influxdb_client.write_points(json_body)
        print(f"led data written to InfluxDB: {led_status}")
    elif msg.topic == MQTT_TOPIC_WATER_PUMP:
            water_pump = int(msg.payload.decode())
            json_body = [
                {
                    "measurement": "water_pump",
                    "fields": {
                        "value": water_pump
                    }
                }
            ]
            influxdb_client.write_points(json_body)
            print(f"pump data written to InfluxDB: {water_pump}")
    elif msg.topic == MQTT_TOPIC_WATER_SENSOR:
        water_container = int(msg.payload.decode())
        json_body = [
            {
                "measurement": "water_container",
                "fields": {
                    "value": water_container
                }
            }
        ]
        influxdb_client.write_points(json_body)
        print(f"water container data written to InfluxDB: {water_container}")
    elif msg.topic == MQTT_TOPIC_FAN:
        fan = int(msg.payload.decode())
        json_body = [
            {
                "measurement": "fan",
                "fields": {
                    "value": fan
                }
            }
        ]
        influxdb_client.write_points(json_body)
        print(f"fan data written to InfluxDB: {fan}")


if __name__ == '__main__': #InfluxDB connection
    influxdb_client = InfluxDBClient(
        host=INFLUXDB_ADDRESS,
        port=INFLUXDB_PORT,
        username=INFLUXDB_USER,
        password=INFLUXDB_PASSWORD,
        database=INFLUXDB_DATABASE
    )

    influxdb_client.create_database(INFLUXDB_DATABASE)

    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

    mqtt_client.loop_forever()
