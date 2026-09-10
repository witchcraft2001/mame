#!/usr/bin/env python3
"""Inject an unpadded 42-byte ARP reply for the pcap RX regression probe."""

import argparse
import fcntl
import os
import platform
import socket
import struct
import time


FRAME = bytes.fromhex(
    "02608cdead42"          # destination: probe station address
    "666574680001"          # source: feth1
    "0806"                  # ARP
    "0001080006040002"      # Ethernet/IPv4 reply
    "666574680001c0a80701"  # sender MAC/IP
    "02608cdead42c0a80781"  # target MAC/IP
)


def send_linux(interface):
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3)) as port:
        port.bind((interface, 0))
        for _ in range(24):
            port.send(FRAME)
            time.sleep(0.25)


def send_macos(interface):
    fd = None
    for number in range(256):
        try:
            fd = os.open(f"/dev/bpf{number}", os.O_RDWR)
            break
        except OSError:
            pass
    if fd is None:
        raise OSError("no writable /dev/bpf device")
    try:
        fcntl.ioctl(fd, 0x8020426C, struct.pack("16s16x", interface.encode("ascii")))
        fcntl.ioctl(fd, 0x80044275, struct.pack("I", 1))  # BIOCSHDRCMPLT
        for _ in range(24):
            if os.write(fd, FRAME) != len(FRAME):
                raise OSError("short BPF write")
            time.sleep(0.25)
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    args = parser.parse_args()
    system = platform.system()
    if system == "Darwin":
        send_macos(args.interface)
    elif system == "Linux":
        send_linux(args.interface)
    else:
        parser.error(f"unsupported platform: {system}")


if __name__ == "__main__":
    main()
