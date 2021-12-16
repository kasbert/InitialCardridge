#!/usr/bin/env python3
# Control program for C64 Initial Cardridge
#
# Don't use spaces in filenames

from __future__ import print_function
from builtins import chr
from builtins import str
from builtins import range
import sys
import serial
import time
import argparse
import textwrap
import os
from modem import *
from modem import tools
import select

parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog=textwrap.dedent('''\
         Commands:
            loadx <file.prg> - Load PRG file to C64 (or use -l <file.prg>)
            run - Start basic program in C64
            reset - perform fast reset
            p - print text
            i - input text
            RESET - perform normal reset
            NMI - perform NMI
            GAME=0 - Set GAME line low (enable)
            GAME=1 - Set GAME line high
            EXROM=0 - Set EXROM line low (enable)
            EXROM=1 - Set EXROM line high
            cd - Change directory in SD
            load <file.prg> - Load PRG file to C64 from SD
            ls - List files in SD
            rm <file> - Remove file in SD
            send <file> - Send file to SD
            recv <file> - Receive file from SD
            <file> - Send and run PRG file in C64
            /<file> - Load PRG file from SD and run it in C64
         '''))
parser.add_argument("-V", "--version", help="version", action="store_true")
parser.add_argument("-d", "--debug", help="debug", action="store_true")
parser.add_argument("-p", "--port", help="device, default /dev/ttyUSB0", default='/dev/ttyUSB0')
parser.add_argument("-W", "--initial-wait", type=float, help="Initial wait before first command", default='1.0')

parser.add_argument("-a", type=int, help="start address", default = None)
parser.add_argument("-c", type=int, help="count", default = 1024)
parser.add_argument("-f", help="file parameter for read DPRAM", default = False)

parser.add_argument("-r", help="read DPRAM to file or stdout(-)")
parser.add_argument("-v", type=argparse.FileType("rb"), help="verify DPRAM with file.bin")
parser.add_argument("-w", type=argparse.FileType("rb"), help="write DPRAM with file.bin")

parser.add_argument("-l", help="load file.prg to C64")
parser.add_argument("-L", type=argparse.FileType("rb"), help="load file.prg to C64 slow")
parser.add_argument("-X", type=argparse.FileType("wb"), help="read mem from C64")
parser.add_argument("-T", type=int, nargs=2, help="transfer addr len")


parser.add_argument('args', nargs=argparse.REMAINDER, help="commands or *.prg")

parser.add_argument("-moncommands", help="ignore vice command")
parser.add_argument("-autostartprgmode", type=int, help="ignore vice command")

RECSIZE = 16
NL = chr(10)

def calcwriteline(a, l):
    ck = 0
    s = "W" + ("%04x" % a) + ":"
    for c in l:
        s = s + ("%02x" % c)
        ck = ck ^ c
    s = s.ljust(RECSIZE * 2 + 6,'f')
    if (len(l) & 1):
        ck = ck ^ 0xff
    ck = ck & 0xff
    s = s + "," + ("%02x" % ck)
    return s.upper()

def calcwritelineHex(a, l):
    ck = len(l) + a + (a >> 8) 
    s = ":" + ("%02x" % len(l)) + ("%04x" % a) + "00"
    for c in l:
        s = s + ("%02x" % c)
        ck = ck + c
    ck = (-ck) & 0xff
    s = s + ("%02x" % ck)
    return s.upper()

def parseRecord(l):
    rom = bytes.fromhex(l[1:].strip())
    ck = 0
    for c in rom:
        ck = ck + c
    val = rom[-1]
    ck = ck & 0xff
    if (ck):
        raise Exception("chksum " + str(val) + "!" + str(ck))
    return rom[4:-1]

class C64Ctrl:

 def __init__(self, serialclient = None, **kwargs):
    ''' Initialize a serial client instance
    '''
    self.debug = kwargs.get('debug', False)
    if serialclient == None:
        port = kwargs.get('port', '/dev/ttyUSB0')
        baudrate = kwargs.get('baudrate', 115200)
        ## Prevent Arduino boot
        #os.system('stty -F ' + port + ' -hupcl')
        self.ser = serial.Serial(port, baudrate, timeout=2.5)
    else:
        self.ser = serialclient
    time.sleep(kwargs.get("initial_wait")) # Opening serial port boots the Arduino

    timeo = self.ser.timeout
    self.ser.timeout = 0.1
    while True:
        s = self.ser.readline()
        if self.debug:
            print('<', repr(s))
        if s is None or len(s) == 0:
            break
    self.ser.timeout = timeo

 def send_cmd(self, s):
    if self.debug:
        print('>', repr(s))
    self.ser.write((s + NL).encode())

 def recv_answer(self):
    s = self.ser.readline()
    s = s.decode()
    if self.debug:
        print('<', repr(s))
    s = s.strip()
    return s

 def cmd(self, s, w=False):
    self.send_cmd(s)
    if w:
        s = self.waitokay()
    else:
        for i in range(50):
            s = self.recv_answer()
            if s != '':
                break
    return s

 def waitokay(self):
    bad = 0
    while True:
        s = self.recv_answer()
        if s == "OK":
            return s
        if s.startswith('ERR'):
            raise Exception(s)
        print(s)
        if s == '':
            bad = bad + 1
        if bad > 50:
            raise Exception("TIMEOUT")

 def read_ram(self, dumpstart, count, f):
    l = self.cmd("R%04x%04x" % (dumpstart, count))
    while True:
        if l == '':
            l = self.recv_answer()
        if l.startswith('OK'):
            return l
        if l.startswith('ERR'):
            raise Exception(l)
        #if l == ':00000001FF':
        #    print l

        rom = parseRecord(l)
        if f:
            f.write(rom)
            f.flush()
            print(l.upper(), "\r", end='')
            sys.stdout.flush()
        else:
            print(l.upper())
        dumpstart = dumpstart + RECSIZE
        count = count - RECSIZE
        l = ''

    if f:
        print()

 def write_ramOld(self, a, f):
    while True:
        l = f.read(16)
        if len(l) == 0:
            break
        s = calcwriteline(a, l)
        if self.debug != True:
            print(s, "\n", end='')
            sys.stdout.flush()
        cmd(s, True)
        if len(l) != 16:
            break
        a = a + 16
    f.close()
    print()

 def write_ram(self, a, f):
    while True:
        l = f.read(RECSIZE)
        if len(l) == 0:
            break
        s = calcwritelineHex(a, l)
        if self.debug != True:
            print(s, "\n", end='')
            sys.stdout.flush()
        self.send_cmd(s)
        if len(l) != RECSIZE:
            break
        a = a + RECSIZE
    self.cmd(":00000001FF", True)
    f.close()
    print()

 def verify_ram(self, f, a):
    badcount = 0
    while True:
        r = f.read(RECSIZE)
        if len(r) == 0:
            break
        okay = 1

        l = self.cmd("R%04x%04x" % (a, RECSIZE))
        self.waitokay()
        rom = parseRecord(l)
        print(l, "RAM", "\r", end='')
        if self.debug:
            print()
        sys.stdout.flush()

        filet = calcwritelineHex(a, r) + " FILE"
        markt = "         "
        for i in range(len(r)):
            if rom[i] != r[i]:
                okay = 0
                badcount = badcount + 1
                markt = markt + '* '
            else:
                markt = markt + '  '

        if okay == 0:
            #print
            print(markt)
            print(filet, end='')
            print("MISMATCH!!")
            #raise Exception()

        if len(r) != RECSIZE:
            break
        else:
            a = a + RECSIZE

    print()
    print(badcount, "errors!")
    f.close()
    return False

 def load_prg(self, f, addr): # Slow load row by row
    print('LOAD ADDRESS', addr)
    while True:
        l = f.read(RECSIZE)
        size = len(l)
        if size == 0:
            break
        s = calcwriteline(0x100, l)
        self.cmd(s, True)
        print('LOAD ADDRESS', addr, size, '\r', end='')
        sys.stdout.flush()
        self.cmd("T%04x%02x" % (addr, size), True)
        addr = addr + size
        if size != RECSIZE:
            break
    print('LOAD ADDRESS', addr)
    # Update basic pointers
    l = bytes([addr & 0xff, addr >> 8, addr & 0xff, addr >> 8, addr & 0xff, addr >> 8])
    s = calcwriteline(0x100, l)
    self.cmd(s, True)
    self.cmd("T%04x%02x" % (0x2d, len(l)), True)
    f.close()
    time.sleep(0.1)
    self.cmd('EXROM=1') # EXROM off
    print()

 def setBasicEnd(self, addr):
    print('LOAD ADDRESS', addr)
    # Update basic pointers
    l = bytes([addr & 0xff, addr >> 8, addr & 0xff, addr >> 8, addr & 0xff, addr >> 8])
    s = calcwriteline(0x100, l)
    self.cmd(s, True);
    self.cmd("T%04x%02x" % (0x2d, len(l)), True)
    print()

 def printText(self, text):
    self.cmd("p " + text, True);

 def inputText(self, text):
    if text == '':
        text = "\r"
    text = text.encode('latin-1').decode('unicode_escape')
    for i in range(0, len(text), 9):
        txt = text[i:i+9]
        textlen = len(txt)
        s = calcwriteline(0x100, txt.encode('latin-1'))
        self.cmd(s, True);
        self.cmd("T%04x%02x" % (0x0277, textlen), True)
        self.cmd(calcwriteline(0x100, bytes([textlen])), True);
        self.cmd("T%04x%02x" % (0xc6, 1), True)
    print()

 # XModem methods

 def getc(self, size, timeout=5):
    r, w, e = select.select([self.ser.fileno()], [], [], timeout)
    if r:
        data = self.ser.read(size)
        return data

 def putc(self, data, timeout=1):
    r, w, e = select.select([], [self.ser.fileno()], [], timeout)
    if w: return self.ser.write(data)

 def recvFileX(self, filename):
    self.send_cmd("sx "+ filename)
    answer = self.recv_answer()
    toks = answer.split(' ') # Don't use spaces in filenames
    if len(toks) != 3 or toks[0] != 'rx':
        print("<", answer)
        print("Error in receiving file")
        return
    size = int(toks[2]) # toks[1] ~= filename
    filename = os.path.basename(filename)
    print("RECEIVE", filename)
    stream = open(filename, 'wb')
    modem = XMODEM(self.getc, self.putc)
    modem.recv(stream)
    if size:
        stream.truncate(size)
    answer = self.recv_answer()
    if answer != 'OK':
        print("Error", repr(answer))
        return

 def sendFileX(self, filename):
    stream = open(filename, 'rb')
    size = os.stat(filename).st_size
    filename = os.path.basename(filename)
    print("SEND", filename, size)
    self.send_cmd("rx "+ filename + " " + str(size))
    modem = XMODEM(self.getc, self.putc)
    modem.send(stream)
    answer = self.recv_answer()
    if answer != 'OK':
        print("Error", repr(answer))
        return

 def loadFileX(self, filename, a):
    size = os.stat(filename).st_size
    f = open(filename, 'rb')
    # size = os.fstat(f.fileno()).st_size
    if a is None:
        l = f.read(2)
        addr = l[0] + (l[1] << 8)
    else:
        if filename.endswith('.prg') or filename.endswith('.PRG'):
            f.read(2)
        addr = a
    print("LOAD", filename, addr, size)
    self.send_cmd("loadx "+ str(addr) + " " + str(size))
    answer = self.recv_answer()
    print("<", answer)
    modem = XMODEM(self.getc, self.putc)
    modem.send(f)
    f.close()
    answer = self.recv_answer()
    if answer.startswith('ERR'):
        print("Error", repr(answer))
        return
    print(answer)
    return addr + size

#
#
#

def main():

    opts = parser.parse_args()
    c64 = C64Ctrl(port = opts.port, debug = opts.debug, initial_wait = opts.initial_wait)

    if opts.version:
        print(c64.cmd('V'))
        sys.exit()

    #s = sys.argv[0]

    if opts.debug:
        tools.log.setLevel(1)
    else:
        tools.log.setLevel(11)

    if opts.r:
        a = opts.a or 0
        c = opts.c or 1
        if opts.r != '-':
            f = open(opts.f, 'wb')
        else:
            f = False
        c64.read_ram(a, c, f)

    if opts.w:
        f = opts.w
        a = opts.a or 0
        c64.write_ram(a, f)

    if opts.T:
        addr = opts.T[0]
        length = opts.T[1]
        c64.cmd("T%04x%02x" % (addr, length), True)
        print()

    if opts.L:
        f = opts.L
        a = opts.a
        if a is None:
            l = f.read(2)
            addr = l[0] + (l[1] << 8)
        else:
            addr = a
        c64.load_prg(f, addr)

    if opts.l:
        c64.loadFileX(opts.l, opts.a)

    if opts.X:
        f = opts.X
        a = opts.a or 0
        c = opts.c or 1
        while c > 0:
            print('LOAD ADDRESS', a, '\r', end='')
            sys.stdout.flush()
            c64.cmd("F%04x%02x" % (a, RECSIZE), True)
            l = c64.cmd("R%04x%04x" % (0x300, RECSIZE))
            rom = parseRecord(l)
            c64.waitokay()
            if f:
                f.write(rom)
                f.flush()
            c = c - RECSIZE
            a = a + RECSIZE
        f.close()

    if opts.v:
        f = opts.v
        a = opts.a or 0
        c64.verify_ram(f, a)


    while len(opts.args) > 0:
        arg = opts.args.pop(0)
        if arg == 'run': # Start BASIC programs
            c64.cmd("run")
            continue
        if arg == 'p':
            assert len(opts.args) > 0, 'argument is missing'
            text = opts.args.pop(0)
            c64.printText(text)
            continue
        if arg == 'i':
            assert len(opts.args) > 0, 'argument is missing'
            text = opts.args.pop(0)
            c64.inputText(text)
            continue

        if arg == 'reset':
            # fast reset
            c64.cmd('EXROM=0', True)
            c64.cmd("RESET", True)
            time.sleep(0.1)
            c64.cmd('EXROM=1', True)
            time.sleep(0.1)
            continue
        if arg == 'RESET':
            c64.cmd("RESET", True)
            continue
        if arg == 'NMI':
            c64.cmd("NMI", True)
            continue

        if arg == 'GAME=0' or arg == 'GAME=1' or arg == 'EXROM=0' or arg == 'EXROM=1':
            c64.cmd(arg, True)
            continue

        if arg == 'loadx':
            assert len(opts.args) > 0, 'file argument is missing'
            filename = opts.args.pop(0)
            c64.loadFileX(filename, opts.a)
            continue
        if arg == 'load':
            assert len(opts.args) > 0, 'file argument is missing'
            filename = opts.args.pop(0)
            c64.cmd('load ' + filename, True)
            continue
        if arg == 'send':
            assert len(opts.args) > 0, 'file argument is missing'
            filename = opts.args.pop(0)
            c64.sendFileX(filename)
            continue
        if arg == 'recv':
            assert len(opts.args) > 0, 'file argument is missing'
            filename = opts.args.pop(0)
            c64.recvFileX(filename)
            continue
        if arg == 'cd':
            assert len(opts.args) > 0, 'file argument is missing'
            filename = opts.args.pop(0)
            c64.cmd('cd ' + filename, True)
            continue
        if arg == 'rm':
            assert len(opts.args) > 0, 'file argument is missing'
            filename = opts.args.pop(0)
            c64.cmd('rm ' + filename, True)
            continue
        if arg == 'ls':
            c64.cmd("ls", True)
            continue
        if arg == 'tree':
            c64.cmd("tree", True)
            continue

        if os.path.exists(arg):
            if arg.endswith('.sym'):
                print('ignore sym files')
                continue
            c64.cmd("reset")
            addr = c64.loadFileX(arg, opts.a)
            c64.setBasicEnd(addr)
            c64.cmd("run")
            continue

        if arg.startswith('/'):
            c64.cmd(arg ,True)
            continue

        print("Unknown command ", repr(arg))
        parser.print_help()
        break
###

if __name__ == "__main__":
    main()

