#!/usr/bin/python3
""" Dual-EVG UDP command class. Supported commands:
    - Software trigger request
    - Reboot request
"""
import sys
import struct
import socket
from time import time
from argparse import ArgumentParser
from typing import Union

# Default network parameters
IP = "192.168.1.129"
PORT = 58762
# Protocol constants
NET_PACKET_MAGIC = 0xBD018426
SW_TRIG_EVENT_GEN_0 = 0x00001100
SW_TRIG_EVENT_GEN_1 = 0x00001101
SW_REBOOT_CMD_ADDR = 0x00001F00
SW_REBOOT_CMD_SET = [1, 100, 10000]

CMD_LIST = [SW_TRIG_EVENT_GEN_0, SW_TRIG_EVENT_GEN_1]
DEFAULT_SW_EVENT_VALUE = 50


class evg_com:
    """This class provides a simple interface to send UDP commands to the Dual-EVG device"""

    def __init__(self, ip: str, port: int = PORT,
                 timeout: float = 1.5, verbose: bool = False):
        """Initialize the class."""
        self.ip = ip
        self.port = port
        self.nonce = int(time())
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.verbose = verbose

    def _build_packet(self, cmd: int, args: Union[int, list]) -> bytes:
        """Assemble the binary packet according to the EVG protocol."""
        packet = struct.pack("<I", NET_PACKET_MAGIC)
        packet += struct.pack("<I", self.nonce)
        packet += struct.pack("<I", cmd)
        if isinstance(args, int): packet += struct.pack("<I", args)
        elif isinstance(args, list):
            for arg in args: packet += struct.pack("<I", arg)
        else: raise ValueError("args must be an integer or a list of integers")
        self.nonce += 1 # devg would ignore packet with same nonce
        return packet

    def reboot(self) -> int:
        """ Send the reboot request sequence """
        for arg in SW_REBOOT_CMD_SET:
            packet = self._build_packet(SW_REBOOT_CMD_ADDR, arg)
            self.sock.sendto(packet, (self.ip, self.port))
            if arg != SW_REBOOT_CMD_SET[-1]: readback = self.sock.recvfrom(1024)
            else: readback = [packet[:12]] # no response due to reboot (it could be checked with timeout)
            if readback is None or readback[0] != packet[:12]:
                if self.verbose: print("[!] Reboot failed.")
                return -1
        return 0

    def sw_trigger_request(self, event: int, generator: int) -> int:
        """Send software trigger request. Arguments:
        - event: value within [1-255].
        - generator: which of the 2 generator to use [0-1].
        """
        if generator == 0: cmd = SW_TRIG_EVENT_GEN_0
        elif generator == 1: cmd = SW_TRIG_EVENT_GEN_1
        else: raise ValueError("[!] Invalid generator index.")
        packet = self._build_packet(cmd, event)
        self.sock.sendto(packet, (self.ip, self.port))
        if self.verbose: print(f"[Info] raw packet: {packet} sent to {self.ip}:{self.port}")
        readback = self.sock.recvfrom(1024)
        if readback is None or readback[0] != packet[:12]:
            if self.verbose: print("[!] Trigger request ignored.")
            return -1
        return 0

if __name__ == "__main__":
    parser = ArgumentParser(
        prog="Dual-EVG UDP network class",
        allow_abbrev=True,
    )
    parser.add_argument(
        "--debug", "--verbose", "--d", "--v", "-d", "-v", dest="verbose", help="Print additional information.",
        action="store_true", default=False)
    parser.add_argument(
        "--ip", "-ip", dest="ip", help="Device IP address. I.e. 192.168.1.129", type=str, default=IP)
    parser.add_argument(
        "--port", "--p", "-p", dest="port", help="Network port", type=int, default=PORT)
    for i in [0,1]: parser.add_argument(
        f"--event_gen{i}", f"--e{i}", f"-e{i}", dest=f"event_g{i}", type=int,
        default=0, help=f"Event value (generator {i}).")
    parser.add_argument(
        "--reboot", "--r", "-r", dest="reboot", action="store_true", default=False, help="Reboot dual-EVG device.")

    args = parser.parse_args()
    com = evg_com(ip=args.ip, port=args.port, verbose=args.verbose)
    result = 0
    if args.event_g0: result |= com.sw_trigger_request(event=args.event_g0, generator=0)
    if args.event_g1: result |= com.sw_trigger_request(event=args.event_g1, generator=1)
    if args.reboot: result |= com.reboot()
    sys.exit(result)
