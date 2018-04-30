import rospy

from iiwa_msgs.msg import JointPosition, JointQuantity
from iiwa_msgs.srv import SetPathParameters
from geometry_msgs.msg import PoseStamped, Pose

global publisher1, publisher2, rate


from math import pi

joint_pos = None
cart_pos = None

def callback_joint(data):
    global joint_pos
    joint_pos = data.position

def callback_cart(data):
    global cart_pos
    cart_pos = data.pose

def abs_move_joint(a1, a2, a3, a4, a5, a6, a7, degree=True):
    iiwa_msg = JointPosition()

    if degree == True:

        iiwa_msg.position.a1 = a1*pi/180
        iiwa_msg.position.a2 = a2*pi/180
        iiwa_msg.position.a3 = a3*pi/180
        iiwa_msg.position.a4 = a4*pi/180
        iiwa_msg.position.a5 = a5*pi/180
        iiwa_msg.position.a6 = a6*pi/180
        iiwa_msg.position.a7 = a7*pi/180
    else:
        iiwa_msg.position.a1 = a1
        iiwa_msg.position.a2 = a2
        iiwa_msg.position.a3 = a3
        iiwa_msg.position.a4 = a4
        iiwa_msg.position.a5 = a5
        iiwa_msg.position.a6 = a6
        iiwa_msg.position.a7 = a7

    while True:
        iiwa_msg.header.stamp = rospy.Time.now()
        publisher1.publish(iiwa_msg)

        if isinstance(joint_pos, JointQuantity)\
                and abs(joint_pos.a7 - iiwa_msg.position.a7) < 0.01\
                and abs(joint_pos.a6 - iiwa_msg.position.a6) < 0.01 \
                and abs(joint_pos.a5 - iiwa_msg.position.a5) < 0.01 \
                and abs(joint_pos.a4 - iiwa_msg.position.a4) < 0.01 \
                and abs(joint_pos.a3 - iiwa_msg.position.a3) < 0.01 \
                and abs(joint_pos.a2 - iiwa_msg.position.a2) < 0.01  \
                and abs(joint_pos.a1 - iiwa_msg.position.a1) < 0.01\
                :
            break

        rate.sleep()

def abs_move_cart(px, py, pz, ox, oy, oz, ow):
    iiwa_msg = PoseStamped()
    iiwa_msg.pose.position.x = px
    iiwa_msg.pose.position.y = py
    iiwa_msg.pose.position.z = pz
    iiwa_msg.pose.orientation.x = ox
    iiwa_msg.pose.orientation.y = oy
    iiwa_msg.pose.orientation.z = oz
    iiwa_msg.pose.orientation.w = ow

    while True:
        iiwa_msg.header.stamp = rospy.Time.now()
        publisher2.publish(iiwa_msg)

        if isinstance(joint_pos, Pose) \
                and abs(joint_pos.position.x - iiwa_msg.pose.position.x) < 0.01 \
                and abs(joint_pos.position.y - iiwa_msg.pose.position.y) < 0.01 \
                and abs(joint_pos.position.z - iiwa_msg.pose.position.z) < 0.01 \
                and abs(joint_pos.orientation.x - iiwa_msg.pose.orientation.x) < 0.01 \
                and abs(joint_pos.orientation.y - iiwa_msg.pose.orientation.y) < 0.01 \
                and abs(joint_pos.orientation.z - iiwa_msg.pose.orientation.z) < 0.01 \
                and abs(joint_pos.orientation.w - iiwa_msg.pose.orientation.w) < 0.01:
            break

        rate.sleep()



if __name__ == "__main__":

    print "initing node"
    rospy.init_node('rosvm', anonymous=True)
    print "finish init"
    publisher1 = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)
    publisher2 = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)

    subscriber1 = rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, callback_joint)
    subscriber2 = rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback_cart)

    rate = rospy.Rate(10)




    abs_move_joint(-75,0,0,100,0,10,90)
    print "exec"
    abs_move_joint(-76,-9,0,116.8,0,35.8,90)
    print "exec"