
import socket

from time import sleep, time
import gripper
import sys


class Robot(object):
    def __init__(self, joint_speed = 1.0, cart_speed = 80):
        self.s = socket.socket()
        self.s.connect(("192.168.131.18",30001))
        self.BUFFER_SIZE = 1024
        self.isconnected = False
        self.JointPosition = ([None, None, None, None, None, None, None], None)
        self.ToolPosition = ([None, None, None, None, None, None], None)
        self.ToolForce = ([None, None, None], None)
        self.ToolTorque = ([None, None, None], None)
        self.isCompliance = (False, None)
        self.isCollision = (False, None)
        self.isReadyToMove = (False, None)
        self.isMastered = (False, None)
        self.OperationMode = (None, None)
        self.OprationMode = (None, None)
        self.JointAcceleration = (None, None)
        self.JointVelocity = (None, None)
        self.JointJerk = (None, None)

        self.isready = False
        self.set_acceleration(0.5)
        self.set_velocity(0.5)
        self.set_cart_velocity(80)

    def pull_data(self, func):
        pass

    @property
    def x(self):
        return self.ToolPosition[0][0]

    @property
    def y(self):
        return self.ToolPosition[0][1]

    @property
    def z(self):
        return self.ToolPosition[0][2]

    @property
    def a(self):
        return self.ToolPosition[0][3]

    @property
    def b(self):
        return self.ToolPosition[0][4]

    @property
    def c(self):
        return self.ToolPosition[0][5]

    @property
    def a1(self):
        return self.JointPosition[0][0]

    @property
    def a2(self):
        return self.JointPosition[0][1]

    @property
    def a3(self):
        return self.JointPosition[0][2]

    @property
    def a4(self):
        return self.JointPosition[0][3]

    @property
    def a5(self):
        return self.JointPosition[0][4]

    @property
    def a6(self):
        return self.JointPosition[0][5]

    @property
    def a7(self):
        return self.JointPosition[0][6]


    def set_velocity(self, value):
        self.s.sendall("setJointVelocity {}\r\n".format(value))
        receive = ""
        while len(receive) < 9:
            receive += self.s.recv(9)
        print receive

    def set_acceleration(self, value):
        self.s.sendall("setJointAcceleration {}\r\n".format(value))
        receive = ""
        while len(receive) < 9:
            receive += self.s.recv(9)
        print receive

    def set_cart_velocity(self, value):
        self.s.sendall("setCartVelocity {}\r\n".format(value))
        receive = ""
        while len(receive) < 9:
            receive += self.s.recv(9)
        print receive

    def rel_move_cart(self, x=0, y=0, z=0, a=0, b=0, c=0):
        self.s.sendall("MoveXYZABC {} {} {} {} {} {}\r\n".format(x, y, z, a, b, c))
        receive = ""
        while len(receive) < 9:
            receive += self.s.recv(9)
        print receive

    def abs_move_cart(self, x, y, z, a, b, c, lin_ptp="ptp"):
        self.s.sendall("setPositionXYZABC {} {} {} {} {} {} {}\r\n".format(x, y, z, a, b, c, lin_ptp))
        receive = ""
        while len(receive) < 9:
            receive += self.s.recv(9)
        print receive

    def abs_move_joint(self, a1, a2, a3, a4, a5, a6, a7):

        self.s.sendall("setPosition {} {} {} {} {} {} {}\r\n".format(a1, a2, a3, a4, a5, a6, a7))
        receive = ""
        while len(receive) < 9:
            receive += self.s.recv(9)
        print receive

    def get_info(self):
        self.s.sendall("getinfo\r\n")
        sleep(0.2)
        data = self.s.recv(self.BUFFER_SIZE)
        #print data
        last_read_time = time()
        for pack in data.split(">"):  # parsing data pack
            cmd_splt = pack.split()

            if len(pack) and cmd_splt[0] == 'Joint_Pos':  # If it's JointPosition
                tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                if len(tmp) == 7: self.JointPosition = (tmp, last_read_time)

            if len(pack) and cmd_splt[0] == 'Tool_Pos':  # If it's ToolPosition
                tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                if len(tmp) == 6: self.ToolPosition = (tmp, last_read_time)

            if len(pack) and cmd_splt[0] == 'Tool_Force':  # If it's ToolForce
                tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                if len(tmp) == 3: self.ToolForce = (tmp, last_read_time)

            if len(pack) and cmd_splt[0] == 'Tool_Torque':  # If it's ToolTorque
                tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                if len(tmp) == 3: self.ToolTorque = (tmp, last_read_time)

            if len(pack) and cmd_splt[0] == 'isCompliance':  # If isCompliance
                if cmd_splt[1] == "false":
                    self.isCompliance = (False, last_read_time)
                elif cmd_splt[1] == "true":
                    self.isCompliance = (True, last_read_time)

            if len(pack) and cmd_splt[0] == 'isCollision':  # If isCollision
                if cmd_splt[1] == "false":
                    self.isCollision = (False, last_read_time)
                elif cmd_splt[1] == "true":
                    self.isCollision = (True, last_read_time)

            if len(pack) and cmd_splt[0] == 'isReadyToMove':  # If isReadyToMove
                if cmd_splt[1] == "false":
                    self.isReadyToMove = (False, last_read_time)
                elif cmd_splt[1] == "true":
                    self.isReadyToMove = (True, last_read_time)

            if len(pack) and cmd_splt[0] == 'isMastered':  # If isMastered
                if cmd_splt[1] == "false":
                    self.isMastered = (False, last_read_time)
                elif cmd_splt[1] == "true":
                    self.isMastered = (True, last_read_time)

            if len(pack) and cmd_splt[0] == 'OperationMode':  # If OperationMode
                self.OperationMode = (cmd_splt[1], last_read_time)

            if len(pack) and cmd_splt[0] == 'JointAcceleration':  # If it's JointAcceleration
                self.JointAcceleration = (float(cmd_splt[1]), last_read_time)

            if len(pack) and cmd_splt[0] == 'JointVelocity':  # If it's JointVelocity
                self.JointVelocity = (float(cmd_splt[1]), last_read_time)

            if len(pack) and cmd_splt[0] == 'JointJerk':  # If it's JointJerk
                self.JointJerk = (float(cmd_splt[1]), last_read_time)

    def gripper_grip(self):
        gripper.gripper_grip()

    def gripper_release(self):
        gripper.gripper_release()

    def gripper_disable(self):
        gripper.gripper_disable()

    def __del__(self):
        self.s.close()

if __name__ == "__main__":
    print("Initializing robot please wait!")
    r = Robot()
    #r.abs_move_joint(-150, 0, -150, 0, -150, 0, -150)
    #r.abs_move_joint(150,0,150,0,150,0,150)
    #### Change this pose = aruco.get_pose()
    #r.abs_move_joint(0,0,0,0,0,0,0)
    # r.rel_move_cart(z=-150)
    # r.get_info()

    num_args = len(sys.argv)
    print("Got {} args".format(num_args))

    # Check if this is a joint movement command
    if num_args == 8:
        print(sys.argv)

        # Move the robot!
        print("Moving the robot")
        r.abs_move_joint(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5],sys.argv[6],sys.argv[7])
        print("Movement finished")

    # Check if this is an abs cmd
    if num_args == 4:
        print(sys.argv)


        r.get_info()
        print("Current robot position is x:{}, y:{}, z:{}".format(r.x, r.y, r.z))

        rel_xmove = -int(r.x - float(sys.argv[1]))
        rel_ymove = -int(r.y - float(sys.argv[2]))
        rel_zmove = -int(r.z - float(sys.argv[3]))
        print("Desired relative movement is x:{}, y:{}, z:{}".format(rel_xmove, rel_ymove, rel_zmove))

        # Move the robot!
        print("Moving the robot")
        while (abs(rel_xmove) > 10 or abs(rel_ymove) > 10 or abs(rel_zmove) > 10):
            # Clip our movement to a range of 50
            if rel_xmove > 50:
                rel_xmove = 50
            elif rel_xmove < -50:
                rel_xmove = -50
            if rel_ymove > 50:
                rel_ymove = 50
            elif rel_ymove < -50:
                rel_ymove = -50
            if rel_zmove > 50:
                rel_zmove = 50
            elif rel_zmove < -50:
                rel_zmove = -50

            r.rel_move_cart(x=rel_xmove, y=rel_ymove, z=rel_zmove)

            r.get_info()   # Get our new desired relative movement
            rel_xmove = -int(r.x - float(sys.argv[1]))
            rel_ymove = -int(r.y - float(sys.argv[2]))
            rel_zmove = -int(r.z - float(sys.argv[3]))

        print("Movement finished")

        r.get_info()
        print("New robot position is x:{}, y:{}, z:{}".format(r.x, r.y, r.z))

    sys.exit()



# print(sys.argv)
#
# r.get_info()
# print("Current robot position is x:{}, y:{}, z:{}".format(r.x, r.y, r.z))
#
# rel_xmove = -int(r.x - float(sys.argv[1]))
# rel_ymove = -int(r.y - float(sys.argv[2]))
# rel_zmove = -int(r.z - float(sys.argv[3]))
# print("Desired relative movement is x:{}, y:{}, z:{}".format(rel_xmove, rel_ymove, rel_zmove))
#
# # Move the robot!
# print("Moving the robot")
# r.rel_move_cart(x=rel_xmove,y=rel_ymove,z=rel_zmove)
# print("Movement finished")
#
# r.get_info()
# print("New robot position is x:{}, y:{}, z:{}".format(r.x, r.y, r.z))
