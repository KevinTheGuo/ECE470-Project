# Week Two: Deriving and displaying forward kinematics in a robot simulation

## 1. Connecting Python to V-REP
1. Install Python from  [here](https://www.python.org/downloads/release/python-364/)
2. You will also need two use an IDE for Python, we used [PyCharm](https://www.jetbrains.com/pycharm/) but you can use whatever you like
3. Connecting Python to the V-REP simulator are is a onetime process that you will have to do before you can run the provided Python code
4. There is an excellent video that you can use to achieve this that explains it much better than any README file could. [Link to video.](https://www.youtube.com/watch?v=SQont-mTnfM)
5. With that, here are some things to look out for when following the instructions in the video
    a. Make sure you have the right version of V-REP installed. We used educational version. If you have the player version installed then you will not be successful.
    b. Make sure you have the right version of Python installed. We use Python 3 and there may be potential incompatibilities using Python 2  

## 2. Deriving Forward Kinematics for the KUKA iiwa 7
The schematics for the KUKA robot are available [in this PDF](https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_kr_15_sl_en.pdf).
The joint locations and orientations are located on page 10, and relevant robot dimensions are located on page 13. If you want, the maximum and minimum angle restrictions are available on page 12.

From these diagrams and measurements, you can sketch a similar schematic to that shown in "ForwardKinematics.pdf", drawn in the style of ECE470 homework assignments.

To derive forward kinematics, write a quick Python program in Jupyter Notebook, with your expert knowledge of forward kinematics. With this program, you can derive a spatial screw of the robot, which you can then matrix multiply by an array of thetas representing each joint angle, to derive a predicted end position frame for the robot's end effector.

Now that you have calculated out the forward kinematics, use the derived screw in the Python code connected to the V-REP simulator to draw a predicted end effector frame when given a certain set of theta joint angles

## 3. Moving the robot and displaying the predicted end frame in simulation
Our code consists of three main components. First, the VREP API is used to connect to the simulation session through Python. This is done by starting a simulation server, then requesting a clientID for the session.

Next, we enumerate the Kuka's joints by requesting handlers for each joint. All joints are initialized to zero.
Next, the user is prompted for a set of seven joint angles for the final pose. These angles are then used to calculate the forward kinematics of the robot.

In addition to this, a dummy object is used to visualize the frame object
