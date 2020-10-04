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
            <file> - Load and run PRG file in C64
         '''))
parser.add_argument("-V", "--version", help="version", action="store_true")
parser.add_argument("-d", "--debug", help="debug", action="store_true")
parser.add_argument("-p", "--port", help="device, default /dev/ttyUSB0", default='/dev/ttyUSB0')

parser.add_argument("-a", type=int, help="start address", default = None)
parser.add_argument("-c", type=int, help="count", default = 1024)
parser.add_argument("-f", help="file parameter for read DPRAM", default = False)

parser.add_argument("-r", help="read DPRAM to terminal or file")
parser.add_argument("-v", type=argparse.FileType("rb"), help="verify DPRAM with file.bin")
parser.add_argument("-w", type=argparse.FileType("rb"), help="write DPRAM with file.bin")

parser.add_argument("-l", help="load file.prg to C64")
parser.add_argument("-L", type=argparse.FileType("rb"), help="load file.prg to C64 slow")
parser.add_argument("-X", type=argparse.FileType("wb"), help="read mem from C64")
parser.add_argument("-T", type=int, nargs=2, help="transfer addr len")


parser.add_argument('args', nargs=argparse.REMAINDER, help="commands or *.prg")

parser.add_argument("-moncommands", help="ignore vice command")
parser.add_argument("-autostartprgmode", type=int, help="ignore vice command")

opts = parser.parse_args()

ser = serial.Serial(opts.port, 115200, timeout=0.5)
time.sleep(1.0) # Opening serial port boots the Arduino
# stty -F /dev/ttyUSB0 -hupcl

while True:
    s = ser.readline()
    if opts.debug:
        print('<', repr(s))
    if s is None or len(s) == 0:
        break

RECSIZE = 16
NL = chr(10)

dumpstart = -1
dumpend = -1

def calcwritelineHex(a, l):
    ck = len(l) + a + (a >> 8) 
    s = ":" + ("%02x" % len(l)) + ("%04x" % a) + "00"
    for c in l:
        s = s + ("%02x" % c)
        ck = ck + c
    ck = (-ck) & 0xff
    s = s + ("%02x" % ck)
    return s.upper()

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

def send_cmd(s):
    if opts.debug:
        print('>', repr(s))
    ser.write((s + NL).encode())

def recv_answer():
    s = ser.readline()
    s = s.decode()
    if opts.debug:
        print('<', repr(s))
    s = s.strip()
    return s
    
def cmd(s, w=False):
    send_cmd(s)
    if w:
        s = waitokay()
    else:
        for i in range(50):
            s = recv_answer()
            if s != '':
                break
    return s

def waitokay():
    bad = 0
    while True:
        s = recv_answer()
        if s == "OK":
            return s
        if s.startswith('ERR'):
            sys.exit(s)
        else:
            bad = bad + 1
        if bad > 50:
            sys.exit("\nTIMEOUT")

def parseRecord(l):
    rom = bytes.fromhex(l[1:].strip())
    ck = 0
    for c in rom:
        ck = ck + c
    val = rom[-1]
    ck = ck & 0xff
    if (ck):
        sys.exit("chksum " + str(val) + "!" + str(ck))
    return rom[4:-1]

def read_ram(dumpstart, count, f):
    l = cmd("R%04x%04x" % (dumpstart, count))
    while True:
        if l == '':
            l = recv_answer()
        if l.startswith('OK'):
            return l
        if l.startswith('ERR'):
            sys.exit(l)
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

def write_ramOld(a, f):
    while True:
        l = f.read(16)
        if len(l) == 0:
            break
        s = calcwriteline(a, l)
        if opts.debug != True:
            print(s, "\n", end='')
            sys.stdout.flush()
        cmd(s, True);
        if len(l) != 16:
            break
        a = a + 16
    f.close()
    print()

def write_ram(a, f):
    while True:
        l = f.read(RECSIZE)
        if len(l) == 0:
            break
        s = calcwritelineHex(a, l)
        if opts.debug != True:
            print(s, "\n", end='')
            sys.stdout.flush()
        send_cmd(s)
        if len(l) != RECSIZE:
            break
        a = a + RECSIZE
    cmd(":00000001FF", True);
    f.close()
    print()

def verify_ram(f, a):
    badcount = 0
    while True:
        r = f.read(RECSIZE)
        if len(r) == 0:
            break
        okay = 1

        l = cmd("R%04x%04x" % (a, RECSIZE))
        waitokay()
        rom = parseRecord(l)
        print(l, "ROM", "\r", end='')
        if opts.debug:
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
            #sys.exit()

        if len(r) != RECSIZE:
            break
        else:
            a = a + RECSIZE

    print()
    print(badcount, "errors!")
    f.close()

def load_prg(f, addr): # Slow load row by row
    print('LOAD ADDRESS', addr)
    while True:
        l = f.read(RECSIZE)
        size = len(l)
        if size == 0:
            break
        s = calcwriteline(0x100, l)
        cmd(s, True);
        print('LOAD ADDRESS', addr, size, '\r', end='')
        sys.stdout.flush()
        cmd("T%04x%02x" % (addr, size), True)
        addr = addr + size
        if size != RECSIZE:
            break
    print('LOAD ADDRESS', addr)
    # Update basic pointers
    l = bytes([addr & 0xff, addr >> 8, addr & 0xff, addr >> 8, addr & 0xff, addr >> 8])
    s = calcwriteline(0x100, l)
    cmd(s, True);
    cmd("T%04x%02x" % (0x2d, len(l)), True)
    f.close()
    time.sleep(0.1)
    cmd('EXROM=1') # EXROM off
    print()

def getc(size, timeout=5):
    r, w, e = select.select([ser.fileno()], [], [], timeout)
    if r:
        data = ser.read(size)
        return data
        
def putc(data, timeout=1):
    r, w, e = select.select([], [ser.fileno()], [], timeout)
    if w: return ser.write(data)

def recvFileX(filename):
    send_cmd("sx "+ filename)
    answer = recv_answer()
    toks = answer.split(' ') # Don't use spaces in filenames
    if len(toks) != 3 or toks[0] != 'rx':
        print("<", answer)
        print("Error in receiving file")
        return
    size = int(toks[2]) # toks[1] ~= filename
    filename = os.path.basename(filename)
    print("RECEIVE", filename)
    stream = open(filename, 'wb')
    modem = XMODEM(getc, putc)
    modem.recv(stream)
    if size:
        stream.truncate(size)
    answer = recv_answer()
    if answer != 'OK':
        print("Error", repr(answer))
        return

def sendFileX(filename):
    # FIXME check file exists
    stream = open(filename, 'rb')
    size = os.stat(filename).st_size
    filename = os.path.basename(filename)
    print("SEND", filename, size)
    send_cmd("rx "+ filename + " " + str(size))
    modem = XMODEM(getc, putc)
    modem.send(stream)
    answer = recv_answer()
    if answer != 'OK':
        print("Error", repr(answer))
        return

def loadFileX(filename):
    # FIXME check file exists
    size = os.stat(filename).st_size
    f = open(filename, 'rb')
    # size = os.fstat(f.fileno()).st_size
    a = opts.a
    if a is None:
        l = f.read(2)
        addr = l[0] + (l[1] << 8)
    else:
        addr = a
    print("LOAD", filename, addr, size)
    send_cmd("loadx "+ str(addr) + " " + str(size))
    answer = recv_answer()
    print("<", answer)
    modem = XMODEM(getc, putc)
    modem.send(f)
    f.close()
    answer = recv_answer()
    if answer.startswith('ERR'):
        print("Error", repr(answer))
        return
    print(answer)
    return addr + size

def setBasicEnd(addr):
    print('LOAD ADDRESS', addr)
    # Update basic pointers
    l = bytes([addr & 0xff, addr >> 8, addr & 0xff, addr >> 8, addr & 0xff, addr >> 8])
    s = calcwriteline(0x100, l)
    cmd(s, True);
    cmd("T%04x%02x" % (0x2d, len(l)), True)
    print()

#
#
#

if opts.version:
    print(cmd('V'))
    sys.exit()

#s = sys.argv[0]

if opts.debug:
    tools.log.setLevel(1)
else:
    tools.log.setLevel(11)
    
if opts.r:
    dumpstart = opts.a
    count = int(opts.r)
    if opts.f:
        f = open(opts.f, 'wb')
    else:
        f = False
    read_ram(dumpstart, count, f)


if opts.w:
    f = opts.w
    #f = open(opts.s, 'rb')
    a = opts.a
    write_ram(a, f)


if opts.T:
    addr = opts.T[0]
    len = opts.T[1]
    cmd("T%04x%02x" % (addr, len), True)
    print()

if opts.L:
    f = opts.L
    #f = open(opts.L, 'rb')
    a = opts.a
    if a is None:
        l = f.read(2)
        addr = l[0] + (l[1] << 8)
    else:
        addr = a
    load_prg(f, addr)

if opts.l:
    loadFileX(opts.l)

if opts.X:
    f = opts.X
    a = opts.a
    c = opts.c
    while c > 0:
        print('LOAD ADDRESS', a, '\r', end='')
        sys.stdout.flush()
        cmd("F%04x%02x" % (a, RECSIZE), True)
        l = cmd("R%04x%04x" % (0x300, RECSIZE))
        rom = parseRecord(l)
        waitokay()
        if f:
            f.write(rom)
            f.flush()
        c = c - RECSIZE
        a = a + RECSIZE
    f.close()

if opts.v:
    f = opts.v
    a = opts.a
    verify_ram(f, a)


while len(opts.args) > 0:
    arg = opts.args.pop(0)
    if arg == 'run': # Start BASIC programs
        cmd("run")
        continue

    if arg == 'reset':
        # fast reset
        cmd('EXROM=0')
        cmd("RESET")
        time.sleep(0.1)
        cmd('EXROM=1')
        time.sleep(0.1)
        continue
    if arg == 'RESET':
        cmd("RESET")
        continue
    if arg == 'NMI':
        cmd("NMI")
        continue

    if arg == 'GAME=0' or arg == 'GAME=1' or arg == 'EXROM=0' or arg == 'EXROM=1':
        cmd(arg)
        continue

    if arg == 'loadx':
        # FIXME check
        filename = opts.args.pop(0)
        loadFileX(filename)
        continue

    if arg == 'load':
        # FIXME check
        filename = opts.args.pop(0)
        send_cmd('load ' + filename)
        while True:
            answer = recv_answer()
            if answer == 'OK':
                break
            if answer.startswith('ERR'):
                print("Error", repr(answer))
                break
            if answer != '':
                print(answer)
        continue
    if arg == 'send':
        # FIXME check
        filename = opts.args.pop(0)
        sendFileX(filename)
        continue
    if arg == 'recv':
        # FIXME check
        filename = opts.args.pop(0)
        recvFileX(filename)
        continue
    if arg == 'cd':
        # FIXME check
        filename = opts.args.pop(0)
        send_cmd('cd ' + filename)
        answer = recv_answer()
        print(answer)
        answer = recv_answer()
        if answer != 'OK':
            print("Error", repr(answer))
        continue
    if arg == 'rm':
        # FIXME check
        filename = opts.args.pop(0)
        send_cmd('rm ' + filename)
        answer = recv_answer()
        if answer != 'OK':
            print("Error", repr(answer))
        continue
    if arg == 'ls':
        send_cmd("ls")
        while True:
            answer = recv_answer()
            if answer == 'OK':
                break
            if answer.startswith('ERR'):
                print("Error", repr(answer))
                break
            print(answer)
        continue
    if arg == 'tree':
        send_cmd("tree")
        while True:
            answer = recv_answer()
            if answer == 'OK':
                break
            if answer.startswith('ERR'):
                print("Error", repr(answer))
                break
            print(answer)
        continue

    if os.path.exists(arg):
        if arg.endswith('.sym'):
            print('ignore sym files')
            continue
        cmd("reset")
        addr = loadFileX(arg)
        setBasicEnd(addr)
        cmd("run")
        continue

    if arg.startswith('/'):
        send_cmd(arg)
        continue

    print("Unknown command ", repr(arg))
    parser.print_help()
    break
###
