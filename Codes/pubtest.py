import paho.mqtt.client as mqtt
import json
import time

# MQTT parameters
broker_address = "192.168.0.237"  # Use the private IP of AWS instance
broker_port = 1883
topic = "sensor/data"

# Create a MQTT client instance
client = mqtt.Client("Publisher")

# Connect to the MQTT broker
client.connect(broker_address, broker_port)

# Data payload
data_payload = {
    "temperature": 22.5,
    "humidity": 48
}

# Publish data
while True:
    client.publish(topic, json.dumps(data_payload))
    print(f"Data published to topic {topic}")
    time.sleep(5)  # Sends data every 5 seconds
