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

## 2. Moving the robot and displaying the predicted end frame in simulation
Our code consists of three main components. First, the VREP API is used to connect to the simulation session through Python. This is done by starting a simulation server, then requesting a clientID for the session.

Next, we enumerate the KUKA's joints by requesting handlers for each joint. All joints are initialized to zero.
Next, the user is prompted for a set of seven joint angles for the final pose. These angles are then used to calculate the forward kinematics of the robot.

In addition to this, a dummy object is used to visualize the frame object

## 3. Moving the Robot through the Guiding Sphere™

Click on the sphere to select it.
You can now drag it around to represent the desired end position and orientation (pose) of the robot.
The robot should follow the sphere to the new location. In case the sphere is posed out of bounds, the robot just remains in the last position while an error message is printed to the terminal.

## 4. Moving and Collisions of Obstacle Dummies

To demonstrate object collision detection, we used 5 different dummy objects to represent external objects.
Click on a dummy and then click on move object tool. The dummy can be dragged to any position and released.
The robot can then be moved using the method described above. Once moved, if the robot is in collision with any of the
movable, external obstacles, a collision will be reported.

## 5. Self Collisions

When the robot is moved to a new position, it will not only check for collisions with external objects, but also check for collisions with itself.
The robot has a bounding sphere at each joint. If that bounding sphere comes in contact with another joint bounding sphere, the robot is in self
collision. A warning is printed to the terminal. For simplicity each joint is checked against every other joint.








All trademarks are property of kektkg inc. You may gain rights to use them by performing a Happy Chicken Dance™ at exactly 12:00 GMT.
