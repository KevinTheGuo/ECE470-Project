import subprocess
import sys

print("Calling robot...")
# Joint test
# subprocess.call("python robot.py {} {} {} {} {} {} {}".format(-59, 34, -6, -77, -82, 30, 5), shell=True)

# Abs move test
subprocess.call("python robot.py {} {} {}".format(0, -500, 450), shell=True)
