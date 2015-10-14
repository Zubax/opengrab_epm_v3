# NicaDrone OpenGrab EPM688 V3.3 firmware update

OpenGrab Electropermanent Magnet



## Building the firmeware on Debian-based system


Clone the repo:

cd ..../home/git (or any directory you like)

git clone https://github.com/Zubax/opengrab_epm.git

todo: something is missing here

cd opengrab_epm

make

cd build

ls

..

firmware.bin  
firmware.elf  
firmware.hex  

## Flashing firmware over serial in Windows based system

Close J4 force the LPC start a bootloader

Close J3 select to serial bootloader 
alternatively open J3 to start the CAN bootloader

Connect the EPM UART TX and RX to an FTDI cable

Power up the board 

Get the NXP Flash Magic serial flasher
http://www.flashmagictool.com/

Baud Rate: 115200

Interface: ISP

Oscilator: 12Mhz

File: firmware.hex

Start!

## Something exploded?

oops sorry
Hardware:
Andreas@NicaDrone.com
Software:
Pavel@Zubax.com
