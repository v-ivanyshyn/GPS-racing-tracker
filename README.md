# GPS racing tracker
(Hardware: Nordic nRF52840, Quectel LC29HEA, MCP2515; software: Platformio, Arduino framework)

![Components](https://github.com/v-ivanyshyn/GPS-racing-tracker/blob/main/components.jpeg)

The device grabs GPS and CAN bus data and sends it through BLE to the on-phone app (currently RaceChrono is supported).

---

> [!NOTE]
> ### How it's different from other DIY projects:
> - Utilizes dual band 10Hz Quectel LC29HEA module, results in 40cm HDOP
> - The firmware contains built-in UART-over-BLE configuration and statistics feature - you can check the functionality from your phone right in fields, as well as configure it
> - CAN data is sniffed from internal CAN bus, then packed into BLE payload. It reduces the BLE bandwith, so no frames are dropped
> - Because of direct CAN bus connection, non-general OBDII data is available (ex. steering angle, tracktion control activation)
> - Software is optimized for data transmission, proved 10Hz rate for GPS data and 30-40Hz for CAN data
> - The module is packed in small OBDII dongle box and plugged into the car's port, no battery or external power supply needed. GPS requires external antenna, which is a good practivce overall
> - The idea behind was to build a device with good enough performance & features, competitive with advanced commercial products, though for much less costs

---

### List of components:
- XIAO nRF52840 https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html
- XIAO CAN Bus Expansion Board https://www.seeedstudio.com/Seeed-Studio-CAN-Bus-Breakout-Board-for-XIAO-and-QT-Py-p-5702.html
- Quectel LC29HEA Module + external antenna https://aliexpress.com/item/1005009915168712.html 
- DC-DC Converter 12V to 5V https://aliexpress.com/item/32801569565.html
- OBDII connector box

![Assembly](https://github.com/v-ivanyshyn/GPS-racing-tracker/blob/main/assembly%201.jpeg)
![Assembly](https://github.com/v-ivanyshyn/GPS-racing-tracker/blob/main/assembly%202.jpeg)

### Usage guide:
⚠️ Provided that your phone supports Bluetooth Low Energy 4.0/5.0
1. Solder hardware according to the image above
2. Connect XIAO nRF52840 module to your laptop through USB and upload the [firmware binary](https://github.com/v-ivanyshyn/GPS-racing-tracker/blob/main/firmware.UF2) (double-click reset button on XIAO board, let the device storage appear on your PC and drag-n-drop the binary into it). Or build the project in PlatformIO and upload your custom binary
3. Power the device from in-car OBDII port or supply 12V on your desk
4. Install on your phone an app that utilizes UART over BLE. For me the best working one is Bluefruit Connect
5. Open the app and connect to the device (named GPS Racing Tracker), navigate to UART feature, type in "help" and see if the device responds. Then you can type other commands, as well as configure the GPS module directly via PQTM or PAIR commands
6. Buy/install RaceChrono app on your phone, follow these instructions to connect the DIY device: https://racechrono.com/article/faq/how-to-connect-a-wi-fi-gps-receiver
7. When connected successfully you shold see at least GPS data in RaceChrono

CAN bus sniffing works out of the box with **Ford Mustang 2015+**. Other models and car brands require different CAN PIDs and interpretation logic, so they aren't supported in this software (also not all cars expose CAN data to OBDII port).

A couple of configuration steps is needed in RaceChrono to befriend it with incomming CAN traffic.

How it works:
- The device grabs CAN data (steering angle, speed, lon & lat G force, braking pressure, engine load, acceleration pedal position, current gear, traction control involvement) from 7 PIDs of raw CAN traffic (according to this [documentation](https://github.com/v-ivanyshyn/parse_can_logs/blob/master/Ford%20CAN%20IDs%20Summary.md))
- These 7 PIDs fit into hardware filters in MCP2515 chip, so other CAN traffic is skipped. Otherwise, if the filtering is applied on software level, the chip isn't capable to transmit all the traffic, so some frames are dropped
- While these CAN PIDs may be transmitted from the module to RaceChrono app directly, BLE bandwidth isn't sufficient and in result some frames get lost. The solution is to pack the data into two custom CAN frames of 8 bytes each with custom PIDs 0x101 and 0x102 assigned. RaceChrono app provides functionality to unpack the data through custom equations.
- You need to enter these equations in RaceChrono app for mapping to incomming data:
  - **Accelerator position (%)** - PID: 257, Equation: `bytesToInt(raw, 5, 1)`
  - **Brake pressure (bar)** - PID: 257, Equation: `float(bytesToInt(raw, 6, 2))`
  - **Digital** - PID: 258, Equation: `bytesToUInt (raw, 7, 1)` - this channel outputs `1` or `2` when traction control applies
  - **Engine RPM (rpm)** - PID: 257, Equation: `bytesToInt(raw, 2, 2)`
  - **Engine load (%)** - PID: 257, Equation: `bytesToInt(raw, 4, 1)`
  - **Gear** - PID: 258, Equation: `bytesToInt(raw, 6, 1)`
  - **Lateral acceleration (G)** - PID: 258, Equation: `float(bytesToInt(raw, 0, 2))*0.01`
  - **Longitudinal acceleration (G)** - PID: 258, Equation: `float(bytesToInt(raw, 2, 2))*0.01`
  - **Speed (km/h)** - PID: 257, Equation: `bytesToUInt(raw, 0, 2)/360`
  - **Steering angle (°)** - PID: 258, Equation: `bytesToInt(raw, 4, 2)`
- After that if you connect the device to your Mustang and toggle "Test connection" in RaceChrono, all the channels should be filled in with live data

### References:
- Quectel_LC29H_Series&LC79H(AL)_GNSS_Protocol_Specification_V1.3.pdf (available on www.quectel.com/ for logged in users)
- Dual frequency RTK for less than $60 with the Quectel LC29HEA https://rtklibexplorer.wordpress.com/2024/04/28/dual-frequency-rtk-for-less-than-60-with-the-quectel-lc29hea/
- Configuring the Quectel LC29HEA receiver for real-time RTK solutions https://rtklibexplorer.wordpress.com/2024/05/06/configuring-the-quectel-lc29hea-receiver-for-real-time-rtk-solutions/
- RaceChronoDiyBleDevice example https://github.com/timurrrr/RaceChronoDiyBleDevice
- RaceChrono BLE DIY APIs https://github.com/aollin/racechrono-ble-diy-device
- GPS lib: https://github.com/mikalhart/TinyGPSPlus
- CAN bus lib: https://github.com/autowp/arduino-mcp2515

### Further features and improvements:
- Support more phone apps (reference to https://github.com/renatobo/bonogps)
- Universal OBDII polling (reference to https://github.com/sandeepmistry/arduino-OBD2)
- Power save mode
