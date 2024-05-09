#!/usr/bin/env python
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************

import os
import sys
import json
import sys, tty, termios
from dynamixel_sdk import *                 # Uses Dynamixel SDK library

def ping(dev):
    print(dev + " ")

    with open('id_list.json') as f:
        di = json.load(f)

    r_list = sorted(di["r-line"])
    l_list = sorted(di["l-line"])
    c_list = sorted(di["c-line"])

    if os.name == 'nt':
        print('os name')
        return 0
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch


# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0

# Define the proper baudrate to search DYNAMIXELs. Note that XL320's baudrate is 1 M bps.
    BAUDRATE                = 4000000

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME                  = dev

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
    if not portHandler.openPort():
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

# Set port baudrate
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

# Try to broadcast ping the Dynamixel
    dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)

    id_list = []
    for dxl_id in dxl_data_list:
        id_list.append(int(dxl_id))
        id_list = sorted(id_list)

    if(id_list == r_list):
        print("foodlyright")
    if(id_list == l_list):
        print("foodlyleft")
    if(id_list == c_list):
        print("foodlycenter")

# Close port
    portHandler.closePort()

    return 1
