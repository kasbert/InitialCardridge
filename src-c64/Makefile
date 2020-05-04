all: browser control browser.h control.h

ARDUINO=$(HOME)/arduino-1.8.12

control:
	cl65 -t c64 -C ./c64-asm.cfg control.s -o control.bin
#	xa control.s -o control.bin

browser:
	cl65 -t c64 -Oi screen.c browser.c dir.c base.c icif.c minidelay.s -o browser.prg

control.h: control.bin
	xxd -i control.bin > control_bin.h

browser.h: browser.prg
	xxd -i browser.prg > browser_prg.h

clean:
	rm -rf *.o browser.prg control.bin *.hex control_bin.h browser_prg.h

#not used
control.hex: control.bin
	# Add size to the beginning
	(perl -e 'print pack('v', -s "control.bin" )' ; cat control.bin) | bin2hex.py --offset=0x30000 - > control.hex

#not used
browser.hex: browser.prg
	# Add size to the beginning
	(perl -e 'print pack('v', -s "browser.prg" )' ; cat browser.prg) | bin2hex.py --offset=0x30400 - > browser.hex

#not used
program: control.hex browser.hex
	# Load code to top part of mega2560 flash
	-$(ARDUINO)/hardware/tools/avr/bin/avrdude -C$(ARDUINO)/hardware/tools/avr/etc/avrdude.conf -v -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:control.hex
	$(ARDUINO)/hardware/tools/avr/bin/avrdude -C$(ARDUINO)/hardware/tools/avr/etc/avrdude.conf -v -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:browser.hex


