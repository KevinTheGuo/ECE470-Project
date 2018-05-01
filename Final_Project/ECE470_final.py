# This file, written in Python3, opens a webcam and analyzes the input frame-by-frame, calculating the pose
# of any Aruco marker detected. Currently it can only check the pose of one at a time. If it finds a pose, it will
# use inverse kinematics to calculate the joint angles required to move the Kuka to there. If the inverse kinematics
# calculation converges, this code will then open robot.py as a subprocesses, using that code, written in Python2, to
# move the robot joints by passing it command-line arguments.

# Import CV2 with importlib magic
import importlib.util
spec = importlib.util.spec_from_file_location("cv2", "/home/lfsony/.local/lib/python3.5/site-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)

# Import subprocess to call python2 code robot.py
import subprocess

# Import our other helper files
import MarkerPose
import inverse_kinematics

# Try to grab video input
video = cv2.VideoCapture(0)
# Exit if video not opened.
if not video.isOpened():
    print('Could not open video!')
    exit()

    try:
        while(True):
            # Check for user input
            key = cv2.waitKey(1) & 0xff
            if key == 27:
                break

            # Grab another frame
            read_success, frame = video.read()
            if not read_success:
                print('Cannot read video file!')
                break

            # Find the marker pose and draw stuff.
            frame, isValid, pose = findAndDrawMarkers(frame)
            cv2.imshow("ECE470 Final Project", frame)

            # If we found a marker pose and converged to it, then move the robot there!
            if (isValid != -1):
                theta_list = inverse_kinematics.inverse_kinematics(pose)  # inverse kinematics
                if theta_list is not None:          # Make sure that inverse kinematics has converged.
                    print("Inverse kinematics has converged! First theta list: ")
                    for i in range(7):
                        print("theta_", i+1, "is", theta_list[i])
                    sleep(1)

                    # Call robot.py with specific command-line arguments, to move the robot to those joint angles
                    subprocess.call(["robot.py", theta_list[0], theta_list[1], theta_list[2], theta_list[3], theta_list[4], theta_list[5], theta_list[6]])

                    sleep(20)   # sleep for a while

    except (Exception, KeyboardInterrupt) as e:
        video.release()
        cv2.destroyAllWindows()
        raise e

    video.release()
    cv2.destroyAllWindows()
