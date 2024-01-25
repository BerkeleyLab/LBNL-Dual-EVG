#! /usr/bin/python3

# Load a script of marble_mmc commands to the target device via UART

# Goals:
#   1. Handle esc/backspace gracefully
#   2. Show both input and output
#   3. Sleep a bit between lines to allow blocking functions with output to return
#       3a. Try reducing the sleep and seeing what happens

import argparse
import serial
import os
import time
from concurrent.futures import ThreadPoolExecutor as Executor

INTERCOMMAND_SLEEP = 0.01 # seconds
POST_SLEEP = 1.0 # seconds

# A global log of read lines
_log = []
_done = False


class LoadError(Exception):
    def __init__(self, s):
        super().__init__(s)


class StreamSerial():
    def __init__(self, port = None, baud = 115200, flush=True):
        self._ready = False
        self.line = None
        if (port == None):
            return None
        try:
            self.dev = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self._ready = True
            if flush:
                self.flush()
        except FileNotFoundError as e:
            print(e)
            self._ready = False

    def flush(self):
        """Read and discard anything in the incoming buffer."""
        if self._ready:
            self.dev.read_all()

    def failed(self):
        return not self._ready

    def writeline(self, line):
        if not self._ready:
            return False
        line = line.encode('ascii')
        self.dev.write(line)
        return True

    def readline(self):
        if not self._ready:
            return None
        buf = []
        timeout = 10
        while True:
            try:
                r = self.dev.read()
            except serial.serialutil.SerialException as e:
                raise LoadError("Device likely in use by another reader.\r\n" + str(e))
            if not r:
                timeout -= 1
                if timeout == 0:
                    return ""
                continue
            else:
                r = r.decode('utf-8')
                #print("r = {}".format(r))
                buf.append(r)
                if r in ('\n', '\r'):
                    self.line = ''.join(buf)
                    if self.line.strip():
                        # prevent breaking on empty lines
                        break
        return self.line

    def getline(self):
        return self.line

    def close(self):
        if hasattr(self, 'dev') and hasattr(self.dev, 'close'):
            self.dev.close()

def readDevice(sdev, wait_on, do_print=False, do_log=False):
    while True:
        if wait_on.done():
            print(">   done")
            break
        line = sdev.readline()
        # readline returns None on device open fail
        # Returns empty string on timeout
        if line is None:
            break
        if len(line) > 0:
            line = line.strip()
            if do_print:
                print(line)
            if do_log:
                _log.append(line)
    print(">   closing")
    sdev.close()
    _done = True
    return True

def getLines(filename):
    if not os.path.exists(filename):
        print("File {} does not exist".format(filename))
        return None
    lines = []
    with open(filename, 'r') as fd:
        line = True
        while (line):
            line = fd.readline()
            if (line):
                lines.append(line)
    return lines

def serveFile(sdev, filename):
    lines = getLines(filename)
    #return serveLines(sdev, lines)
    return serveCommands(sdev, lines)

def serveCommands(sdev, *commands):
    nlines = 0
    for line in commands:
        if len(line) > 0 and not line.strip().startswith('#'):
            # Bread if writeline returns False
            if not sdev.writeline(line + '\r\n'):
                break
            nlines += 1
            time.sleep(INTERCOMMAND_SLEEP)
    time.sleep(POST_SLEEP)
    print(f">   Wrote {nlines} lines")
    return

def testReadLines(argv):
    USAGE = "python3 {} scriptname".format(argv[0])
    if len(argv) < 2:
        print(USAGE)
        return False
    filename = argv[1]
    lines = getLines(filename)
    print("got {}".format(lines))
    return True

def get_log():
    timeout = 100
    while not (task1.done() and task2.done()):
        time.sleep(1)
        if timeout == 0:
            print("Timeout waiting on task1 and task2")
            break
        else:
            timeout -= 1
    return _log

def loadCommands(dev, baud=115200, commands=None, do_print=False, do_log=False):
    if commands is None:
        print("Missing mandatory filename")
        return 1
    sdev = StreamSerial(dev, baud)
    if sdev.failed():
        return 1
    executor = Executor(max_workers = 2)
    global task1, task2
    task1 = executor.submit(serveCommands, sdev, *commands)
    task2 = executor.submit(readDevice, sdev, task1, do_print, do_log)
    return 0

def loadFile(dev, baud=115200, filename=None):
    if filename is None:
        print("Missing mandatory filename")
        return 1
    sdev = StreamSerial(dev, baud)
    if sdev.failed():
        return 1
    executor = Executor(max_workers = 2)
    global task1, task2
    task1 = executor.submit(serveFile, sdev, filename)
    task2 = executor.submit(readDevice, sdev, task1)
    return 0

def ArgParser():
    parser = argparse.ArgumentParser(description="Script loader for marble_mmc")
    parser.add_argument('-d', '--dev', default=None,
                        help='Device descriptor of TTY/COM port for marble_mmc')
    parser.add_argument('-b', '--baud', default=115200, help="UART Baud rate")
    return parser

def doLoad(argv):
    # Unused 'argv' since argparse is handling everything
    parser = ArgParser()
    parser.add_argument('-f', '--filename', default=None, help='File name for command script to be loaded')
    # Any ordered args will be assumed to be commands to pass to device
    parser.add_argument("commands", nargs=argparse.REMAINDER, help="Strings to be sent directly to device")
    args = parser.parse_args()
    if args.filename is None and len(args.commands) == 0:
        print("Missing mandatory filename or ordered args")
        return 1
    if args.filename is not None:
        loadFile(args.dev, args.baud, args.filename)
    elif len(args.commands) > 0:
        loadCommands(args.dev, args.baud, args.commands, do_print=True)
    return 0

if __name__ == "__main__":
    import sys
    sys.exit(doLoad(sys.argv))
    #testReadLines(sys.argv)
