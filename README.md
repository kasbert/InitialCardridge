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
 * Control C64 from computer via USB
   * Upload PRG file from host computer to C64
   * Control SD card files
     * Upload a file from host computer to SD card
     * Download file from SD card to host computer
     * ls - LiSt files
     * rm - ReMove file
     * cd - Change Dir

C64 and host computer communicate using ASCII protocol commands
 * Commands are terminated with CR or LF
 * Command response is "OK" or "ERR..."
 * "ls"
 * "tree"
 * "cd <dirname>"
 * "pwd"
 * "rm <filename>"
 * "rx <filename> <size>" - Store file to SD. Use XModem in transfer.
 * "sx <filename> <size>" - Fetch file fom SD. Use XModem in transfer.
 * "load <filename>" - Load file from SD to C64
 * "run"
 * "p <text>" - Slow print to C64 screen
 * "autosleep <secs>" - Sleep in playlist mode
 * "sleep <secs>" - Sleep in playlist mode
 * "lc" - Load control program to DPRAM
 * "lb" - Load browse to C64
 * "reset" - Fast C64 reset
 * "RESET" - Trigger C64 reset
 * "NMI" - Trigger C64 NMI
 * "GAME=<num>" - Set GAME pin state
 * "EXROM=<num>" - Set EXROM pin state
 * "loadx <size>" - Load file from host to C64. Use XModem in transfer.
 * "V" - Show version
 * "R<four hex char address><four hex char size>" - Read <size> bytes from DPRAM, beginning at address
 * "W<four hex char address>:<hex char data>..." - Write some bytes to DPRAM
 * ":..." - Write Intel hex file to DPRAM
 * "T<four hex char address><two hex char size>" - Transfer bytes from DPRAM to C64
 * "F<four hex char address><two hex char size>" - Transfer bytes from C64 to DPRAM
 * "/<filename>" - Load file from SD to C64 and run it.
