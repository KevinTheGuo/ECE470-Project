# This files contains code which uses opencv to access a webcam and grab frames for some processing.
# Currently, it's set up to call a function from MarkerPose.py

import importlib.util
spec = importlib.util.spec_from_file_location("cv2", "/home/lfsony/.local/lib/python3.5/site-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)

import MarkerPose

# Try to grab video input
video = cv2.VideoCapture(1)
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

        # Draw the markers, and maybe their poses
        frame, isValid, pose = MarkerPose.findAndDrawMarkers(frame)
        cv2.imshow("Aruco-Time!", frame)

except (Exception, KeyboardInterrupt) as e:
    video.release()
    cv2.destroyAllWindows()
    raise e

video.release()
cv2.destroyAllWindows()
