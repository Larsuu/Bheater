# Bheater
a DIY Battery ECU & firmware.

Software:
Supported Networking Protocols:
- Wifi/MQTT => Home Assistant
- BluetoothSerial

Hardware:
- ESP32-D0WD
- 2 PID controlled PWM output (N-fet)
    Voltage: 25V - 100V 
    Amps:    ~5A

- Dallas DS18B20 temp sensor
- Charger Control (P-fet) ~5A
- Ignition Control (P-fet) 
