#!/bin/env python3

import usb.core
import usb.util
import struct
from matplotlib import pyplot as plt

dev = usb.core.find(idVendor=0xdead, idProduct=0xbeef)

if dev is None:
    raise ValueError("Device not found")

dev.set_configuration()

cfg = dev.get_active_configuration()

intf = cfg[(0, 0)]

ep_control = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(
    e.bEndpointAddress) == usb.util.ENDPOINT_OUT)


ep_data = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(
    e.bEndpointAddress) == usb.util.ENDPOINT_IN)


assert ep_control is not None
assert ep_data is not None

print("connected")

def start_capture(freq):
    packet = struct.pack("<bI", 0x01, freq)
    ep_control.write(packet)


def stop_capture():
    packet = struct.pack("<bI", 0x00, 0x00)
    ep_control.write(packet)


def print_values():
    while True:
        packet = ep_data.read(64, timeout=0xffffffff)
        n = len(packet)

        lost = struct.unpack("<I", packet[n - 4:n])
        lost = lost[0]
        if lost != 0:
            print("Lost {} samples".format(lost))


sample_idx = 0

start_capture(1000)

plt.ion()

y = [[], [], [], []]
x = [[], [], [], []]

while True:
    packet = ep_data.read(64, timeout=0xffffffff)
    n = len(packet)

    lost = struct.unpack("<I", packet[n - 4:n])
    lost = lost[0]
    if lost != 0:
        sample_idx += lost
        print("Lost {} samples".format(lost))

    print("plot")

    plt.clf()

    for i in range(0, 4):
        y[i] += list(map(lambda v: (v >> i) & 1,  packet[:60]))
        x[i] += range(sample_idx, sample_idx + 60)
        plt.plot(x[i], y[i])


    plt.pause(0.02)

    sample_idx += n - 4
