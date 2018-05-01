import subprocess
import sys

print("Calling robot...")
# Joint test
# subprocess.call("python robot.py {} {} {} {} {} {} {}".format(-59, 34, -6, -77, -82, 30, 5), shell=True)

# Abs move test
subprocess.call("python robot.py {} {} {}".format(-50, 0, 0), shell=True)
# subprocess.call("python robot.py {} {} {}".format(0.0, -20.0, -60.0), shell=True)
