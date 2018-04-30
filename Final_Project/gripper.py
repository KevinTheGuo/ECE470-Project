from __future__ import print_function

import os
import sys
from serial.tools import list_ports
from time import sleep



# Add parent directory to path so the example can run without the library being installed
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import serial
import rs485

# USB HW IDs
RS485_VID = 0x1A86
RS485_PID = 0x7523

# This device's ID
DEVICE_ID = "MM1"

# Scan attached devices and find RS485 adapter
device_list = list_ports.comports()
usb_port = None
for device in device_list:
    if device.vid == RS485_VID and device.pid == RS485_PID:
        usb_port = device.device
        break
    usb_port = None

# Check if device was found
if usb_port == None:
    print("No RS485 to USB adapter found!")
    raise Exception
    sys.exit()

# Open non-blocking communication using above usb port
com_port = serial.Serial(usb_port, baudrate=28800, timeout=0, rtscts=True)
rs485 = rs485.SerialWrapper(com_port)

prev_packet = ""

def send_packet_check(packet):
    packet = "MGR GRIPPER0" + packet
    prev_packet = packet
    rs485.sendMsg(packet)
    sleep(0.1)  # Wait for slave to prepare to respond

    for check in range(5):
        print("Checking for incoming data")
        if rs485.update():
            packet = str(rs485.getPacket().decode())

            # Check if the packet is for this device
            if packet[:3] == DEVICE_ID:
                sender_id = packet[3:6]
                print("Received: \"", packet[6:], "\" from device ", sender_id, sep='')
            else:
                print("Received packet for", packet[:3]);

            # Stop checking
            return True

    return False

def gripper_release():
    packet = "SET 200"
    while not send_packet_check(packet):
        pass

    return

def gripper_grip():
    packet = "TORQUE 600, 40"

    while not send_packet_check(packet):
        pass

    return

def gripper_disable():
    packet = "DISABLE"

    while not send_packet_check(packet):
        pass

    return

if __name__ == "__main__":

    try:
        while True:
            gripper_grip()
            sleep(5)
            gripper_release()
            sleep(5)

    finally:
        gripper_disable()

    """
    while True:
        packet = raw_input("Enter packet to send: ")
        if packet == "":
            packet = prev_packet
        else:
            packet = "MGR GRIPPER0" + packet
        prev_packet = packet
        rs485.sendMsg(packet)
        sleep(0.1)  # Wait for slave to prepare to respond

        for check in range(5):
            print("Checking for incoming data")
            if rs485.update():
                packet = str(rs485.getPacket().decode())

                # Check if the packet is for this device
                if packet[:3] == DEVICE_ID:
                    sender_id = packet[3:6]
                    print("Received: \"", packet[6:], "\" from device ", sender_id, sep='')
                else:
                    print("Received packet for", packet[:3]);

                # Stop checking
                break
    """
