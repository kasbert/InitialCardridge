# InitialCardridge
Multipurpose cardridge for Commodore 64

Hardware: https://easyeda.com/jarkko.sonninen/initial-cardridge-bidir

Cardridge consists of 1kB dual port SRAM memory and Arduino Mega 2560 Pro and a SD card reader.

Dual port RAM is used for communication between Arduino and C64. Both computers can access the RAM and it is used for storing pieces of code and data transfer buffers. C64 see the RAM in module area ($8000-$9FFF) and I/O 2 ($DF00). The latter is used by C64 to deliver messages to Arduino.

Arduino is connected to cardridge port pins RESET, NMI, GAME and EXROM and it controls C64 with these.

If Arduino USB is connected to a computer, the link may be used for transferring data from computer to C64 or SD card.

SD card may be used for storing C64 software.

Features of current software:
 * Browser for SD card
   * Load and run PRG files
   * Open D64 disk images and load and run PRG files
   * Traverse directory structure
     * Arduino keeps track of current directory
 * Pressing RESTORE quickly three times resets C64 and loads SD browser
 * Upload PRG file from host computer to C64
 * Control SD card files
   * Upload a file from host computer to SD card
   * Download file from SD card to host computer
   * ls - LiSt files
   * rm - ReMove file
   * cd - Change Dir
 
