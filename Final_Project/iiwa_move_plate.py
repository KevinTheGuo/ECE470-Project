#!/usr/bin/env python

from client_lib import *
import gripper

# Making a connection object.
client = kuka_iiwa_ros_client()

# Wait until iiwa is connected zzz!
while (not client.isready): pass
print('Started!')

"""
# global safe

client.send_command('setPosition -70 10 0 100 0 0 90')

# machine safe
client.send_command('setPosition 90 10 0 100 0 0 90')

# to hotel1 safe
client.send_command("MoveXYZABC 0 100 140 0 0 0")
client.send_command("MoveXYZABC 0 0 0 0 90 0")
client.send_command("MoveXYZABC 0 0 200 0 0 0")

# to nest
client.send_command("MoveXYZABC 0 300 0 0 0 0")
"""

def go_to_global_safe():
    client.send_command('setPosition -70 10 0 100 0 0 90')

def go_to_hotel():
    # global safe
    client.send_command('setPosition -75 0 0 100 0 10 90')

    # to hotel1 safe
    client.send_command("setPosition -76 -9 0 116.8 0 35.8 90")
    #client.send_command("MoveXYZABC 15 0 0 0 0 0")
    client.send_command("MoveXYZABC 0 0 0 0 104.1 0")
    client.send_command("MoveXYZABC 0 0 10 0 0 0")
    client.send_command("MoveXYZABC 0 0 0 0 0 0")

def from_hotel_to_nest_safe():
    # to nest
    #client.send_command("MoveXYZABC 0 -25 0 0 0 0")
    client.send_command("setPositionXYZABC -181.85 496.17 526.10 -90 0 -90 lin")
    client.send_command("setPositionXYZABC 42.19 493.96 524.10 -90 0 -90 lin")

def go_to_machine_safe():
    client.send_command("setPosition 90 10 0 100 0 0 90")
    #client.send_command("setPosition 90 -27 0 77 0 14 90")

def from_nest_safe_to_nest():
    client.send_command("setPositionXYZABC 42.26 493.92 509.69 -90.02 0 -90 lin")
    #client.send_command("MoveXYZABC 0 0 155 0 0 0")
    #client.send_command("MoveXYZABC 0 9 0 0 0 0")

def from_nest_to_nest_safe():

    client.send_command("MoveXYZABC 0 -19 0 0 0 0")
    client.send_command("MoveXYZABC 0 0 -155 0 0 0")

def from_hotel_to_global_safe():

    client.send_command("MoveXYZABC 0 0 -10 0 0 0")
    client.send_command("MoveXYZABC 0 0 0 0 -104.1 0")


    go_to_global_safe()

def to_drop_plate():
    #client.send_command("MoveXYZABC 0 500 0 0 0 0")
    client.send_command("setPositionXYZABC 0 -768 330 - 0 - lin")
    client.send_command("setPositionXYZABC 0 -767.83 158.50 - 0 - lin")



if __name__ == "__main__":
    # Initializing Tool 1
    client.send_command('setTool tool1')

    # Initializing
    client.send_command('setJointAcceleration 1.0')  # If the JointAcceleration $
    client.send_command('setJointVelocity 0.4')  # If the JointVelocity is n$
    client.send_command('setJointJerk 1.0')

    client.send_command('setCartVelocity 100')

    go_to_global_safe()
    #sleep(5)
    #gripper_release()
    #sleep(2)

    go_to_hotel()
    #sleep(5)
    from_hotel_to_nest_safe()
    #sleep(5)
    from_nest_safe_to_nest()
    #sleep(5)

    #gripper_grip()
    #sleep(2)

    from_nest_to_nest_safe()
    #sleep(5)
    from_hotel_to_global_safe()
    #sleep(8)

    go_to_machine_safe()
    #sleep(10)
    to_drop_plate()
    #sleep(10)

    #gripper_release()
    #sleep(2)
    #gripper_grip()
    #sleep(2)

    go_to_machine_safe()
    #sleep(10)

    go_to_global_safe()
    #sleep(8)
    #####gripper_release()

    go_to_hotel()
    #sleep(5)
    from_hotel_to_nest_safe()
    #sleep(8)
    from_nest_safe_to_nest()
    #sleep(8)

    #gripper_release()
    #sleep(5)

    from_nest_to_nest_safe()
    #sleep(10)
    from_hotel_to_global_safe()
    #sleep(5)




