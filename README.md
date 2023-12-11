

# Bheater - A DIY Battery ECU & firmware.

This project was originally designed for small electric vehicles, offering essential control over key battery functions such as charging status, battery heating, and an additional heater. It includes extra buttons for custom features.

**Why**

Living in Finland, I noticed a significant issue. In the cold weather, the battery performance was not optimal. By heating the battery to 25 degrees Celsius, I found that it not only provided more power but also maintained a stable voltage.

Key Features:

  - Wireless Voltage Meter: Check your battery level through Bluetooth or Home Assistant.
  - Voltage Control: Set your preferred charging voltage level to save battery cycles, extending battery life by up to 1.5 times.
  - Remote Charger Switch: Control the charging remotely, either locally through Bluetooth or even through WAN with Home Assistant.

(Status: Basic functionality around heating and charging is written and done. All the extra features are still missing)

Software Features
- Charger Status Control: Manages the status of the battery charger.
- Battery Heater Control: Provides control over a heater used to maintain the battery temperature.
- Auxiliary Heater Control: An additional heater control for optional features like a bench warmer or grip warmer.
- GPIO: Additional 3V3 tolerant GPIO for user-managed features or optional hardware functionality.

Hardware
- ESP32-D0WD MCU
- Dallas DS18B20 sensor
- Charger Control (P-fet): Controls the charging of the battery, with a capacity of around 5A.
- Ignition Control (P-fet): Manages the ignition system.
- 2 PID controlled PWM output (N-fet): Two outputs controlled by a PID (Proportional-Integral-Derivative) controller provide precise voltage and current control.
- The N-fet voltage range is 25V - 100V and the current capacity is around 5A.

Connectivity
- Wifi/MQTT: Connects to Home Assistant for smart home integration.
- BluetoothSerial: Provides Classical Bluetooth connectivity for basic command-line interface control.
- Android App (coming): An upcoming Android app will provide a user-friendly interface for controlling and monitoring the system. (Link to the Velluz app)

