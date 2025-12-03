import time
from awscrt import mqtt
from awsiot import mqtt_connection_builder

# AWS IoT configuration
AWS_IOT_ENDPOINT = "a3lkzcadhi1yzr-ats.iot.eu-west-1.amazonaws.com"
AWS_IOT_CLIENT_ID = "fov-python-publisher"  # Different from C5!
MQTT_TOPIC = "dalymount_IRL/pub"

# Certificate paths
CA_CERT = "ca.crt"
DEVICE_CERT = "device.crt"
DEVICE_KEY = "device.key"

mqtt_connection = mqtt_connection_builder.mtls_from_path(
    endpoint=AWS_IOT_ENDPOINT,
    cert_filepath=DEVICE_CERT,
    pri_key_filepath=DEVICE_KEY,
    ca_filepath=CA_CERT,
    client_id=AWS_IOT_CLIENT_ID,
    clean_session=False,
    keep_alive_secs=30
)

print("Connecting to AWS IoT...")
connect_future = mqtt_connection.connect()
connect_future.result()
print("Connected!")

counter = 0

try:
    while True:
        # Send just the number - no JSON
        message = str(counter)
        mqtt_connection.publish(
            topic=MQTT_TOPIC,
            payload=message,
            qos=mqtt.QoS.AT_LEAST_ONCE
        )
        print(f"Published: {counter}")
        counter += 1
        time.sleep(0.1)  # 10 FPS

except KeyboardInterrupt:
    print("\nDisconnecting...")
    disconnect_future = mqtt_connection.disconnect()
    disconnect_future.result()
    print("Disconnected!")