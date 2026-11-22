ESP8266 considerations
======================

* NO TCP/IP
* NO Buttons
* Only one (build in) Led
* Fastloaders are not tested.

Install ESP8266 gcc. See https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/linux-setup.html
Fetch ESP8266_NONOS_SDK from https://github.com/espressif/ESP8266_NONOS_SDK
Point environment variable ESP_SDK to the SDK directory.
Install esptool.

Current configuration expects 32Mbit (4MB) flash memory.

I used Wemos D1 mini with the following pinout
 GPIO5 = SCL - IEC CLOCK
 GPIO4 = SDA - IEC DATA
 GPIO0 - IEC ATN (pulled up, connected to FLASH button, boot fails if pulled LOW)
 GPIO16 - IEC SRQ (no interrupt, HIGH at boot)
 GPIO2 - Internal LED (pulled up, LED on low)
 GPIO3 = RX - (TODO Button 1 ?)
 GPIO1 = TX - (TODO LED 2 ?)
 ADC0 - (TODO Button 2 ?)
 GPIO 12(MISO),13(MOSI),14(SCLK),15(CS) SD card
 See https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
You need a level converter as ESP8266 is a 3.3V device.

Some of the flash is used as a FAT image. You can create one with linux commands:
 dd if=/dev/zero bs=512 count=7112 of=eeprom.img
 mformat -i eeprom.img -v internal
 mcopy -i eeprom.img /usr/local/share/vice/DRIVES/dos1541 ::
 mcopy -i eeprom.img ../warez/santa.prg ::
 esptool write_flash 0x81000 eeprom.img

Save rom image name
OPEN 15,8,15
PRINT#15,"XR:dos1541"

Internal "ATA" drive is the default drive to use. To use SD as default, issue commands:
OPEN 15,8,15
PRINT#15,"XD0=4"
PRINT#15,"XD1=0"
PRINT#15,"XW"
