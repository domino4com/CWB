<img src="assets/CWB.svg" width=200 align="right">

# CWB Core WiFi Battery
This is the standalone battery core used in the Domino4 eco-system.

## Main ChipSet
The core is built around the RISC-V [ESP32-C3](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf) MCU ChipSet from Espressif. 

This core has a few important features compared to the other Domino4 cores:
- As the name suggest, it features a CR3220 coin cell battery for low power applications, such as reading a sensor maybe once an hour and upload data over WiFi once a day.
- It has a USB-C connection directly on the board, allowing for both powering the circuit and programming the core. The USB-C is directly intergrated into the core so it allows for various USB protocols, such as communicate directly with a keyboard/mouse. See the many Arduino examples, when you install the board file for this core. (Same board file as for the other ESP32 cores, just make sure it is the latest version)
- Similar to the Extended Core (CWV), it has an expansion board, with same configuration as the CWV's expansion board. So this core can run SPI modules directly, like SD Card, LoRa radios, or Ethernet PoE.
- Because of limitation of pins, it uses a multi color Neopixel LED.
- It has a single push button.


## Pin Usage
### Extension Slot
The 10 pins on the Extension slot are configured on both side, making your extension board reversible.
| Pins| Function | Group |Lora|
|:-----------------------------:|:----:|:--:|:--:|
| :one:                 | Vcc | Power|Vcc
| :two:                 | MISO | SPI|MISO
| :three:                    | MOSI | SPI|MOSI
| :four:                  | SCK | SPI|SCK
| :five:            | IO4  | GPIO|NSS
| :six: | IO0 | GPIO|DIO0
| :seven:                       | IO1 | GPIO| ---
| :eight:                       | SCL | I²C | n/a
| :nine:                       | SDA | I²C | n/a
| :keycap_ten:                       | GND | Power| GND

### SD Card
SD Card is used in 4 Pin SPI configuration.
| Pin | SPI (4 pin) | (6 pin)|
|:-----------------------------|:----:|:----:|
|  8| MISO |DAT0| 
|  7| MOSI |CMD| 
|  10| SCK |CLK| 
|  4| CS |CD/DAT3| 
|  0|  |DAT1|
|  1|  |DAT2|

> In Arduino: Choose ```ESPC3 Dev Module``` and use the PPU</br>
> Then find this example: ```Examples``` :arrow_right: ```Examples for ESPC3 Dev Module```  :arrow_right: ```SD``` :arrow_right: ```SD_Test```</br>
> Change this line ```if(!SD.begin()){``` :arrow_right: ```if(!SD.begin(4)){```

### Other Pins
| Function |  GPIO | Notes|
|:-----------------------------|:----:|:--|
|  I²C SDA |5| |
|  I²C SCL |6| |
|  Serial TX |21| |
|  Serial RX |20| |
|  USB + |17| |
|  USB - |18| |
|  Neopixel |2 |  |
|  Button |3 |  |
|  IO |0| On the Bus, but also available on the extension slot |

## Programming

### Programming in Arduino
- To program the Domino4 cores using Arduino, install the board files using the doumentation from [Espressif](https://github.com/espressif/arduino-esp32)
- You can program the core both over the USB and using the PPU. They show up as different Serial ports.
  - Please read the notes regarding upload/transfer speed when using the [PPU](https://github.com/domino4com/PPU).
  - **Speed:** Max 460800 bps, if you are using the PPU. With USB, the speed doesn't matter.
  - **Port:** Chose the port where your PPU is inserted, or where the USB is connected. If you cannot see the port, the check out your [PPU installation](https://github.com/domino4com/PPU)
- You can use two settings:
  - **Board:** Choose the ```ESP32C3 Dev Module```
    - With this board you have to set the I²C pins: ```Wire.setPins(5,6);```
    - Use the PPU to print to Serial. You can use ```ESP_LOGE("Some short text", "Some long text");``` to print to serial on USB.
  - **Board:** Choose the ```Adafruit QT Py ESP32-C3```
    - With this board you need to redirect your serial connection if you use the PPU: ```#define Serial Serial0```
 

### Programming in Python.
- Download the MicroPython firmware from [micropython.org](https://micropython.org/download/esp32c3/)
- It is recommended to download and use the (Mu Editor)(https://codewith.mu/en/download)
- You can use the Mu Editor to upload the MicroPython Firmware.

### Hardware Modification:
- On the back of the xchip, is a solder pad called ```LED_ON```. If you don't like the blue power LED, you can carefully cut the connection between the two pads. And you can naturally reverse that by soldering them together again.

### I²C
I²C's SDA and SCL is not on the standard ESP32's Pin normally used in Arduino. The pins might have to be set before the `Wire.begin()` statement like this:
```C
#define I2C_SDA 5
#define I2C_SCL 6
void setup() {
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
}
```
### SPI

#### Example: LoRa over SPI
I suggest using this library [RadioLib](https://github.com/jgromes/RadioLib), choose examples using ```SX127x```
```C
SX1278 radio = new Module(4, 0,RADIOLIB_NC,RADIOLIB_NC);
```

#### Example: SD Card over SPI
```C
#include "SD.h"
#include "SPI.h"

#define SD_CS 4

void setup() {
  if (!SD.begin(SD_CS)) {
  // Error code
  }
}
```

#### Example: Ethernet (and MQTT) over SPI
```C
#include <EthernetENC.h>
#include <SPI.h>
#include "PubSubClient.h"

uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x06};
EthernetClient ethClient;
PubSubClient mqttClient;
#define ETHERNET_CS 4
void setup() {
  Ethernet.init(ETHERNET_CS);
  delay(200);
  if (Ethernet.begin(mac) == 0) {
    // Fail
  }

  mqttClient.setClient(ethClient);
  mqttClient.setServer( MQTT_SERVER, 1883); 
  reconnect();
}
void reconnect() {
  mqttClient.connect(CLIENTID, USERNAME, PASSWORD);
}
void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  } else {
    mqttClient.publish(TOPIC, MESSAGE);
  }
  mqttClient.loop();
  delay(3000);
}
```

## Troubleshooting
- If you try to upload code and getting this message ```A fatal error occurred: Timed out waiting for packet content``` or ```A fatal error occurred: Invalid head of packet (0xE0)```, change the transfer speed to 460800 pbs.
- If you try to upload code and getting this message ```[7886] Failed to execute script esptool the selected serial port [7886] Failed to execute script esptool does not exist or your board is not connected```, your serial port is open by another application. Close the other application and try again.
- Your code could brick the xchip from using the USB-C connection to flash your software. In that case use a PPU and erase your xchip.

# License: 
<img src="assets/CC-BY-NC-SA.svg" width=200 align="right">
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License

[View License Deed](https://creativecommons.org/licenses/by-nc-sa/4.0/) | [View Legal Code](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

