#!/usr/bin/env python

import errno
import socket
import sys
import time
import struct


class UdpSocket(object):
    """A UDP socket."""

    def __init__(self, device, blocking=True, is_input=True):
        a = device.split(":")
        if len(a) != 2:
            print("UDP ports must be specified as host:port")
            sys.exit(1)
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if is_input:
            self.port.bind((a[0], int(a[1])))
            self.destination_addr = None
        else:
            self.destination_addr = (a[0], int(a[1]))
        if not blocking:
            self.port.setblocking(0)
        self.last_address = None

    def recv(self, n=1000):
        try:
            data, self.last_address = self.port.recvfrom(n)
        except socket.error as e:
            if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK]:
                return ""
            raise
        return data

    def write(self, buf):
        try:
            if self.destination_addr:
                self.port.sendto(buf, self.destination_addr)
            else:
                self.port.sendto(buf, self.last_addr)
        except socket.error:
            pass


class SimData:
    def __init__(self, buf):
        self._fmt = []

        # Get variables
        self.VERSION_EXPECTED = 0
        self.getInt("version")
        self.getDouble("pos_n")
        self.getDouble("pos_e")
        self.getDouble("pos_d")
        self.getDouble("home_lat")
        self.getDouble("home_lng")
        self.getDouble("home_alt")
        self.getDouble("lat")
        self.getDouble("lng")
        self.getDouble("alt")
        self.getFloat("agl")
        self.getFloat("phi")
        self.getFloat("theta")
        self.getFloat("psi")
        self.getFloat("alpha")
        self.getFloat("beta")
        self.getFloatArray("rpm", 2)

        self.data = self.build(buf)

    @staticmethod
    def parse(buf):
        return SimData(buf).data

    def getInt(self, var):
        self._fmt.append((var, "i", 1))

    def getFloat(self, var):
        self._fmt.append((var, "f", 1))

    def getDouble(self, var):
        self._fmt.append((var, "d", 1))

    def getFloatArray(self, var, length):
        self._fmt.append((var, "f", length))

    def build(self, buf):
        fmt = ["="]
        for _, f, l in self._fmt:
            for i in range(l):
                fmt.append(f)
        unpacked = struct.unpack("".join(fmt), buf)
        data = {}
        idx = 0
        for var, _, l in self._fmt:
            if l == 1:
                data[var] = unpacked[idx]
            else:
                data[var] = [unpacked[idx + i] for i in range(l)]
            idx += l
        if data["version"] != self.VERSION_EXPECTED:
            raise "Schema version mismatch"
        return data


if __name__ == "__main__":
    udp = UdpSocket("127.0.0.1:5507")
    tlast = time.time()
    print("Waiting for connection")
    while True:
        buf = udp.recv(1024)
        if time.time() - tlast > 1.0:
            data = SimData.parse(buf)
            print(
                "home_alt",
                data["home_alt"],
                "alt",
                data["alt"],
                "agl",
                data["agl"]
            )
            tlast = time.time()
