# Week Three: Demonstrating Inverse Kinematics

## 1. Connecting Python to V-REP
### These are the same initial steps as the past weeks'
1. Install Python from  [here](https://www.python.org/downloads/release/python-364/)
2. You will also need two use an IDE for Python, we used [PyCharm](https://www.jetbrains.com/pycharm/) but you can use whatever you like
3. Connecting Python to the V-REP simulator are is a onetime process that you will have to do before you can run the provided Python code
4. There is an excellent video that you can use to achieve this that explains it much better than any README file could. [Link to video.](https://www.youtube.com/watch?v=SQont-mTnfM)
5. With that, here are some things to look out for when following the instructions in the video
    a. Make sure you have the right version of V-REP installed. We used educational version. If you have the player version installed then you will not be successful.
    b. Make sure you have the right version of Python installed. We use Python 3 and there may be potential incompatibilities using Python 2  

## 2. Calculating inverse kinematics
We calculate inverse kinematics through an iterative, numerical method. The rough steps are as follows:
1. Make first theta guess of robot position
2. Find the current pose of the robot end effector given the current theta guess
3. Determine a spatial twist to align the robot end effector frame with the desired frame with 1 second
4. Find the space Jacobian as a function of current theta
5. Perform the inverse velocity kinematics, and find thetadot. Use the new algorithm which works on robots with more/less than 6 joints!
6. Calculate the new set of theta angles for the robot joints by applying our calculated thetadot for 1 second
7. Repeat steps 2-6 until the norm of our spatial twist is below a specified cutoff!
The exact functions and code are found in the inverse_kinematics.py file in this folder.

## 3. Taking in desired position and moving the robot
Our code consists of several major components. First, as in the previous week, use the VREP API to start a simulation server and connect to the simulation session through Python. First, the VREP API is used to connect to the simulation session through Python. Next, grab handlers for each of the Kuka's joints, as well as the user-movable dummy. All the Kuka's joints are initialized to zero. The user may now click on the generated dummy to select it and change its position and orientation. This is the robot's new desired pose. The robot will, in realtime, attempt to calculate a set of thetas to move its end effector to the user-selected destination, as the user moves the dummy around. If the destination position and orientation are unreachable, the robot will alert you that "I'm sorry Tim, I'm afraid I can't do that."
