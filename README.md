# Photon-WaterTank-Monitor
Water tank monitor to measure the water level of a tank or cistern

The water level of the cistern / tank is measured using a HC-SR04 ultrasonic sensor. They are not really build for the wet and humid environments like a water tank, but they are cheap and so far I have made good experiences with it. The sensor works in the third season now (I remove them from the tank during winter time).

The results (in cm) are stored internally and published regularly to Particle Cloud, MQTT and my Influx DB (via HTTP).

Readings are taken on regularly basis (*MEASUREMENT_INTERVAL*) and compared with the previous measurement. I no change occurs the system goes into sleep mode to save energy. I changes occur because of a lot of water taken out or much rain, the reading interval is reduced (*MEASUREMENT_INTERVAL_SHORT*) to capture the changes.

The system runs on battery and is optimized for battery life. Under good conditions it runs ~ 3-4 month with a 2.000mAh LiPo battery. This time is mainly influence by the publishing interval (*PUBLISH_INTERVAL*) because connecting to WiFi consumes by far the most energy, especially under difficult conditions like inside of a water cistern (ours has a cover of steel) with weak Wifi.
