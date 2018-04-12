# ECE470 project
### kguo10, edvall2, kthayyi2

#### Assignment 3.05- Displaying robot in V-REP
#### Assignment 3.12- Calculating and Demonstrating Forward Kinematics in V-REP
#### Assignment 3.27- Calculating and Demonstrating Inverse Kinematics in V-REP

## Week One: Instructions on running a robot simulation
1. Download V-REP from [here](http://www.coppeliarobotics.com/downloads.html)
2. Run the installer (make sure you have the recommended disk space)
3. In the models tree, open Robots → non-mobile
4. Scroll Down to find the UR3 robot.
5. Drag two UR3 robots to the platform.
6. In the pane on the left most side of the window, click on the scripts button.
7. This will open a window that shows all scripts that will be run when you hit the play button
8. While the simulation is stopped, double-click on each of the robots' scripts to open and edit it
9. Copy the code for each robot into the corresponding script
10. Close the script editing windows and the "script" windows
11. Click "Play" on the top bar and watch as your robots come to life and mirror each other in real time!

## Week Two: Deriving and displaying forward kinematics in a robot simulation

### 1. Connecting Python to V-REP
1. Install Python from  [here](https://www.python.org/downloads/release/python-364/)
2. You will also need two use an IDE for Python, we used [PyCharm](https://www.jetbrains.com/pycharm/) but you can use whatever you like
3. Connecting Python to the V-REP simulator are is a onetime process that you will have to do before you can run the provided Python code
4. There is an excellent video that you can use to achieve this that explains it much better than any README file could. [Link to video.](https://www.youtube.com/watch?v=SQont-mTnfM)
5. With that, here are some things to look out for when following the instructions in the video
    a. Make sure you have the right version of V-REP installed. We used educational version. If you have the player version installed then you will not be successful.
    b. Make sure you have the right version of Python installed. We use Python 3 and there may be potential incompatibilities using Python 2  

### 2. Deriving Forward Kinematics for the KUKA LBR iiwa 7 R800
1. The schematics for the KUKA robot are available [in this PDF](https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_kr_15_sl_en.pdf).
The joint locations and orientations are located on page 10, and relevant robot dimensions are located on page 13. If you want, the maximum and minimum angle restrictions are available on page 12.

2. From these diagrams and measurements, you can sketch a similar schematic to that shown in "ForwardKinematics.pdf", drawn in the style of ECE470 homework assignments.

3. To derive forward kinematics, write a quick Python program in Jupyter Notebook, with your expert knowledge of forward kinematics. With this program, you can derive a spatial screw of the robot, which you can then matrix multiply by an array of thetas representing each joint angle, to derive a predicted end position frame for the robot's end effector.

4. Now that you have calculated out the forward kinematics, use the derived screw in the Python code connected to the V-REP simulator to draw a predicted end effector frame when given a certain set of theta joint angles

### 3. Moving the robot and displaying the predicted end frame in simulation
Our code consists of three main components. First, the VREP API is used to connect to the simulation session through Python. This is done by starting a simulation server, then requesting a clientID for the session.

Next, we enumerate the Kuka's joints by requesting handlers for each joint. All joints are initialized to zero.
Next, the user is prompted for a set of seven joint angles for the final pose. These angles are then used to calculate the forward kinematics of the robot.

In addition to this, a dummy object is used to visualize the frame object


## Week Three: Demonstrating Inverse Kinematics

### 1. Connecting Python to V-REP
#### These are the same initial steps as the past week's
1. Install Python from  [here](https://www.python.org/downloads/release/python-364/)
2. You will also need two use an IDE for Python, we used [PyCharm](https://www.jetbrains.com/pycharm/) but you can use whatever you like
3. Connecting Python to the V-REP simulator are is a onetime process that you will have to do before you can run the provided Python code
4. There is an excellent video that you can use to achieve this that explains it much better than any README file could. [Link to video.](https://www.youtube.com/watch?v=SQont-mTnfM)
5. With that, here are some things to look out for when following the instructions in the video
    a. Make sure you have the right version of V-REP installed. We used educational version. If you have the player version installed then you will not be successful.
    b. Make sure you have the right version of Python installed. We use Python 3 and there may be potential incompatibilities using Python 2  

### 2. Calculating inverse kinematics
We calculate inverse kinematics through an iterative, numerical method. The rough steps are as follows:
1. Make first theta guess of robot position
2. Find the current pose of the robot end effector given the current theta guess
3. Determine a spatial twist to align the robot end effector frame with the desired frame with 1 second
4. Find the space Jacobian as a function of current theta
5. Perform the inverse velocity kinematics, and find thetadot. Use the new algorithm which works on robots with more/less than 6 joints!
6. Calculate the new set of theta angles for the robot joints by applying our calculated thetadot for 1 second
7. Repeat steps 2-6 until the norm of our spatial twist is below a specified cutoff!
The exact functions and code are found in the inverse_kinematics.py file in this folder.

### 3. Taking in desired position and moving the robot
Our code consists of several major components. First, as in the previous week, use the VREP API to start a simulation server and connect to the simulation session through Python. First, the VREP API is used to connect to the simulation session through Python. Next, grab handlers for each of the Kuka's joints, as well as the user-movable dummy. All the Kuka's joints are initialized to zero. The user may now click on the generated dummy to select it and change its position and orientation. This is the robot's new desired pose. The robot will, in realtime, attempt to calculate a set of thetas to move its end effector to the user-selected destination, as the user moves the dummy around. If the destination position and orientation are unreachable, the robot will alert you that "I'm sorry Tim, I'm afraid I can't do that."

## Week Four: Demonstrating Collision Checking

### 1. Connecting Python to V-REP
#### These are the same initial steps as the past weeks'
1. Install Python from  [here](https://www.python.org/downloads/release/python-364/)
2. You will also need two use an IDE for Python, we used [PyCharm](https://www.jetbrains.com/pycharm/) but you can use whatever you like
3. Connecting Python to the V-REP simulator are is a onetime process that you will have to do before you can run the provided Python code
4. There is an excellent video that you can use to achieve this that explains it much better than any README file could. [Link to video.](https://www.youtube.com/watch?v=SQont-mTnfM)
5. With that, here are some things to look out for when following the instructions in the video
    a. Make sure you have the right version of V-REP installed. We used educational version. If you have the player version installed then you will not be successful.
    b. Make sure you have the right version of Python installed. We use Python 3 and there may be potential incompatibilities using Python 2  

### 2. Moving the robot and displaying the predicted end frame in simulation
Our code consists of three main components. First, the VREP API is used to connect to the simulation session through Python. This is done by starting a simulation server, then requesting a clientID for the session.

Next, we enumerate the KUKA's joints by requesting handlers for each joint. All joints are initialized to zero.
Next, the user is prompted for a set of seven joint angles for the final pose. These angles are then used to calculate the forward kinematics of the robot.

In addition to this, a dummy object is used to visualize the frame object

### 3. Moving the Robot through the Guiding Sphere™

Click on the sphere to select it.
You can now drag it around to represent the desired end position and orientation (pose) of the robot.
The robot should follow the sphere to the new location. In case the sphere is posed out of bounds, the robot just remains in the last position while an error message is printed to the terminal.

### 4. Moving and Collisions of Obstacle Dummies

To demonstrate object collision detection, we used 5 different dummy objects to represent external objects.
Click on a dummy and then click on move object tool. The dummy can be dragged to any position and released.
The robot can then be moved using the method described above. Once moved, if the robot is in collision with any of the
movable, external obstacles, a collision will be reported.

### 5. Self Collisions

When the robot is moved to a new position, it will not only check for collisions with external objects, but also check for collisions with itself.
The robot has a bounding sphere at each joint. If that bounding sphere comes in contact with another joint bounding sphere, the robot is in self
collision. A warning is printed to the terminal. For simplicity each joint is checked against every other joint.

## Week Four: Demonstrating Path Planning

### 1. Connecting Python to V-REP
#### These are the same initial steps as the past weeks'
1. Install Python from  [here](https://www.python.org/downloads/release/python-364/)
2. You will also need two use an IDE for Python, we used [PyCharm](https://www.jetbrains.com/pycharm/) but you can use whatever you like
3. Connecting Python to the V-REP simulator are is a onetime process that you will have to do before you can run the provided Python code
4. There is an excellent video that you can use to achieve this that explains it much better than any README file could. [Link to video.](https://www.youtube.com/watch?v=SQont-mTnfM)
5. With that, here are some things to look out for when following the instructions in the video
    a. Make sure you have the right version of V-REP installed. We used educational version. If you have the player version installed then you will not be successful.
    b. Make sure you have the right version of Python installed. We use Python 3 and there may be potential incompatibilities using Python 2  

### 2. Moving the robot and displaying the predicted end frame in simulation
Our code consists of three main components. First, the VREP API is used to connect to the simulation session through Python. This is done by starting a simulation server, then requesting a clientID for the session.

Next, we enumerate the KUKA's joints by requesting handlers for each joint. All joints are initialized to zero.
Next, the user is prompted for a set of seven joint angles for the final pose. These angles are then used to calculate the forward kinematics of the robot.

In addition to this, a dummy object is used to visualize the frame object

### 3. Moving the Robot with the Guiding Sphere™

Click on the sphere to select it.
You can now drag it around to represent the desired end position and orientation (pose) of the robot.
The robot should follow the sphere to the new location. In case the sphere is posed out of bounds, the robot just remains in the last position while an error message is printed to the terminal.

### 4. Moving and Collisions of Obstacle Dummies

To demonstrate object collision detection, we used 5 different dummy objects to represent external objects.
Click on a dummy and then click on move object tool. The dummy can be dragged to any position and released.
The robot can then be moved using the method described above. Once moved, if the robot is in collision with any of the
movable, external obstacles, a collision will be reported.

### 5. Self Collisions

When the robot is moved to a new position, it will not only check for collisions with external objects, but also check for collisions with itself.
The robot has a bounding sphere at each joint. If that bounding sphere comes in contact with another joint bounding sphere, the robot is in self
collision. A warning is printed to the terminal. For simplicity each joint is checked against every other joint.
***
### 6. Path Planning

We start by acquiring the current position of the robot and using inverse kinematics to figure out the θ~start~ set.
The Guiding Sphere(TM) then indicates the X Y Z coordinates of our target location. We again use Inverse Kinematics to turn this into a θ~goal~ set.
We perform the following steps in a loop till we successfully find a path from θ~start~ to θ~goal~.
1.  Generate a random θ set
2.  check if the θ set would be in collision
a. if in collision go back to step 1 and repeat.
b. once you find a valid θ set go to step 3
3. Check the distance of the current θ set and the last node in θ~start~ and θ~goal~.
4. Add the θ set to θ~start~ or θ~goal~, whichever one has the shorter distance.

As we add theta in our configuration-space to either the start or end trees, we will visualize the
position of the end-effector that corresponds to these thetas, with either a green sphere (attached to the
goal tree), or a red sphere (attached to the starting tree)
Finally, once we discover a path from the start-theta tree to the end-theta tree, the robot will iterate
through the waypoints that represent this path, until it reaches the goal pose. The user will then be able
to move the guidance sphere and hit 'enter' to indicate to the robot to repeat the above process
