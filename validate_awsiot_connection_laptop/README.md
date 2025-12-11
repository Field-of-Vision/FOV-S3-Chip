# Validate AWSIOT Connection on Laptop 

This repo primarily contains the `send_mqtt_10fps.py` script which sends increasing integers to an AWS IOT MQTT topic, 
so that we can test if the C5 chip is successfully subscribing and listening for these messages. 

Note that this script will be used with the `S3-C5-SubscribeMQTT` project primarily currently