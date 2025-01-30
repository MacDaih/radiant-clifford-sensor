# Radiant Clifford Sensor
This application is part of the Radiant Clifford weather service. 
Module running on raspberry pi using the WiringPi lib for sensor data collection. It also uses WaveShare Pi hat for barometric pressure measurement and the board's internal temperature.

## Data publication
The module publishes MQTT application messages toward a broker unsing the lib `porter_c_sdk`
