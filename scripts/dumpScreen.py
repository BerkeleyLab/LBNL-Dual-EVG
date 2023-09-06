"""
Marble instrumentation timing projects: dump screen script
Flake8 --ignore=E266,E501,E701,W503,W504,W605
"""

import os
import re
import time
import socket
import argparse
import numpy as np
from PIL import Image
from datetime import datetime
import serial
from serial.serialutil import SerialException
from serial.tools.list_ports_posix import comports


class dumpScreen():
    destination = "/dev/ttyUSB2"  # default destination value
    re_ttyUSB_string = "^(\/dev\/ttyUSB)\d{1,2}"
    re_IP_address = "^(\d{1,3}).\d{1,3}.\d{1,3}.\d{1,3}"
    re_undesired_serial_text = "[a-zA-z].*?\\r"
    dump_screen_command = 'dumpscreen\r\n'.encode('ascii')
    reading_timeout = 120  # seconds
    filename = None

    class wrongDestination(Exception):
        """ Exception to raise in case the destination indicated is neither ttyUSB nor IP address
        """
        error = "[!] Error - destination type is neither ttyUSB nor IP address."

        def __init__(self, *args, **kwargs):
            print(self.error)
            super().__init__(*args, **kwargs)

    class criticalError(Exception):
        """ Exception to raise in case the destination indicated is neither ttyUSB nor IP address
        """
        error = "[!] Critical Error - "

        def __init__(self, *args, **kwargs):
            for arg in args:
                self.error += str(arg)
            print(self.error)
            super().__init__(*args, **kwargs)

    def __init__(self, destination: str = None, port: int = 50004, filename: str = '', verbose: bool = False) -> None:
        """
        Args:
            destination (str, optional): _description_. Defaults to /dev/ttyUSB2.
            filename (str, optional): image filename. Defaults is 'current_datetime'.bmp.
            verbose (bool, optional): print additional information. Defaults to False.
        """
        if destination is None:  # use default value
            if verbose: print(f"[INFO] destination not specified. Using default value '{self.destination}'.")
        elif type(destination) is not str:
            raise self.wrongDestination
        else:
            self.destination = destination
        if filename is not None and len(filename) > 0:
            self.filename = filename
        self.verbose = verbose
        self.port = port
        self.screenRes = [0, 0]
        self.destinationAnalysis()

    def destinationAnalysis(self):
        """ Distinguish to use serial or socket depending on specified destination
        """
        serial_match = re.fullmatch(self.re_ttyUSB_string, self.destination)
        ip_match = re.fullmatch(self.re_IP_address, self.destination)
        if serial_match:
            if self.verbose: print("[INFO] destination detected as ttyUSB type.")
            self.destination = serial_match[0]
            if not self.serialInit():
                raise self.criticalError("unable to initialize serial communication!.")
        elif ip_match:
            if self.verbose: print("[INFO] destination detected as IP address type.")
            self.destination = ip_match[0]
            if not self.socketInit():
                raise self.criticalError("unable to initialize socket communication!.")
        else:
            raise self.wrongDestination()

    def serialInit(self):
        """ Serial communication initialization
        """
        try:
            self.ser = serial.Serial(port=self.destination,
                                     baudrate=115200,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     bytesize=serial.EIGHTBITS,
                                     timeout=0)
            if self.ser.is_open is False:
                print("[!] Error - serial port not accesible!")
                return False
            else:
                print(f"[Success] Connection established with {self.ser.port} [bd: {self.ser.baudrate}]")
                self.readable = self.ser.readable
                self.in_buffer_len = lambda: self.ser.in_waiting
                self.readline = self.ser.readline
                self.writable = self.ser.writable
                self.write = self.ser.write
                self.flush_channel = self.ser.reset_input_buffer
                return True
        except Exception as e:
            print("[!] Error - something happened during the access to serial resource. ",
                  "Further details:\n", e)

    def socket_readline(self):
        try:
            return self.socket.recv(2048)
        except Exception:
            return ''.encode()

    def socketInit(self):
        """ Socket communication initialization
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.settimeout(5)
            self.readline = self.socket_readline
            self.readable = lambda: os.system("ping -c 1 " + self.destination) == 0
            self.in_buffer_len = lambda: 0  # fake method
            self.writable = self.readable
            self.write = lambda text: self.socket.sendto(text, (self.destination, self.port))
            self.flush_channel = lambda: self.socket.recv(2048)
            print(f"[Success] Connection established with {self.destination}:{self.port}")
            return True
        except Exception as e:
            print("[!] Error - something happened during the access to socket resource. ",
                  "Further details:\n", e)

    def checkInit(self):
        """ Check initialization (serial or socket) via function pointer being not None
        """
        return hasattr(self, 'readable') and \
            hasattr(self, 'in_buffer_len') and \
            hasattr(self, 'readline') and \
            hasattr(self, 'writable') and \
            hasattr(self, 'write') and \
            hasattr(self, 'flush_channel')

    def getIndexAfter(self, buffer: str, pattern: str):
        """ Return index of buffer string just after provided pattern
        """
        offset = len(pattern)
        index = buffer.find(pattern)
        if index >= 0:
            return index + offset
        else:
            return index

    def removeBeforePattern(self, buffer: str, pattern: str):
        """ Discard the portion of buffer string that matches the pattern. """
        index = self.getIndexAfter(buffer, pattern)
        if index >= 0:
            return buffer.split(pattern)[1]
        else:
            return buffer

    def generateFileName(self):
        """ If filename was not provided during class initialization,
            generate a name from current datetime
        """
        if self.filename is not None and len(self.filename) > 0:
            return self.filename
        else:
            return datetime.now().strftime("%Y_%m_%d-%H_%M_%S.bmp")

    def getScreen(self):
        """ Get Marble screen using serial or socket and save it on disk.
        """
        command_feedback = self.dump_screen_command.decode('ascii')
        command_start_pattern = 'P3\r\n'
        screen_info = None
        pixel_buffer = []
        # Command writing and check
        if self.write(self.dump_screen_command) != len(self.dump_screen_command):
            print("[!] Error - write operation error. Please retry.")
            return False
        # Reading
        reading_start_time = time.time()
        watchdog = time.time()
        buffer = bytes()
        while True:
            if time.time() - watchdog > 5: break
            if time.time() - reading_start_time > self.reading_timeout: break
            try:
                data = self.readline()
            except SerialException:
                raise self.criticalError('serial connection not working. The resource may be occupated by another program or process.')
            if data:
                watchdog = time.time()
                buffer += data
        buffer = buffer.decode('ascii')
        if buffer.find('Invalid') != -1 or buffer.find(command_feedback) < 0:
            print("[!] Error - communication error occurred. Please retry.")
            return False
        buffer = self.removeBeforePattern(buffer, command_start_pattern)
        if self.verbose:
            print("[Info] Analysis of input in progress.. \n\tDiscarded lines:")
        for line in buffer.splitlines():
            if screen_info is None:
                screen_info = re.findall('\d{1,3}', line)
                if screen_info[2] == '255':
                    self.screenRes = [int(screen_info[0]), int(screen_info[1])]
                    pixel_counter = int(screen_info[0]) * int(screen_info[1])
            elif not re.findall('[a-zA-Z]', line):
                if len(pixel_buffer) > 3 * pixel_counter:
                    break
                pixel_buffer += [int(res) for res in re.findall('\d{1,3}', line)]
            elif self.verbose:
                print('\t*]\t', line)
        try:
            pixel_buffer = np.reshape(pixel_buffer, (self.screenRes[1], self.screenRes[0], 3)).astype(np.uint8)
        except Exception:
            raise self.criticalError('number of received pixel less than display resolution!')
        img = Image.fromarray(pixel_buffer)
        filename = self.generateFileName()
        try:
            img.save(filename)
        except ValueError:
            print("[!] Error - privided filename has wrong image extension. Using default filename.")
            filename = None; filename = self.generateFileName()
            img.save(filename)
        print(f"[Success] Image saved [{filename}].")
        return True


class cc():
    """ Console color codes """
    RED = '\033[91m'
    GREEN = '\033[92m'
    ORANGE = '\033[93m'
    BLUE = '\033[94m'
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


def showSerialInfo():
    """ Print all serial port available with match to marble serial number (grouped by color)
    """
    color_list = [cc.CYAN, cc.GREEN, cc.ORANGE, cc.RED, cc.PURPLE, cc.BLUE]
    color_index = 0
    last_usb_serial = 0
    dev_iterators = sorted(comports())
    for dev in dev_iterators:
        if 'ttyUSB' in dev.device:
            if int(dev.serial_number) != last_usb_serial:
                if last_usb_serial != 0:
                    color_index = (color_index + 1) % len(color_list)
                last_usb_serial = int(dev.serial_number)
            print(f"Port {color_list[color_index]}{dev.device}{cc.RESET} -> ",
                  f"Board: {cc.BOLD}{dev.product}{cc.RESET}",
                  f":{color_list[color_index]}{dev.serial_number}{cc.RESET}")


def getTTYUSBdetails():
    """ Print serial port that matched with marble boards via linux os command.
    """
    print("List of matches between Marble board serial number and available ttyUSB:")
    if os.system("ls -lR /dev/serial/by-id | grep Marble"):
        print("[!] Error - something went wrong with system command. Check if USB cable is connected.")


if __name__ == '__main__':
    # d = dumpScreen('192.168.1.129', 50004, verbose=True)
    parser = argparse.ArgumentParser(description='Marble screen dump script.')
    parser.add_argument('--board-info', '-i', '-b', dest='info',
                        help='Command to show all information about serial devices',
                        action='store_true', default=False)
    parser.add_argument('--verbose', '-v', dest='verbose',
                        help='Commando to print additional information',
                        action='store_true', default=False)
    parser.add_argument('--device', '-d', dest='device',
                        help='Device to communicate with. I.e. /dev/ttyUSB2 or 192.168.10.120',  # indicate also the port? :xxxx
                        type=str, default='')
    parser.add_argument('--image-filename', '-f', dest='filename',
                        help='Specify the image filename. I.e. EVFR_main_screen.bmp',
                        type=str, default='')
    parser.add_argument('--port', '-p', help='socket port', dest='port', type=int,
                        default=50004)
    args = parser.parse_args()
    if args.info:
        showSerialInfo()
    if len(args.device) > 0:
        dumper = dumpScreen(args.device, args.port, args.filename, args.verbose)
        dumper.getScreen()
