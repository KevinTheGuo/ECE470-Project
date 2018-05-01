import subprocess
import sys

print("Calling robot...")
subprocess.call("python robot.py {} {} {} {} {} {} {}".format(-59, 34, -6, -77, -82, 30, 5), shell=True)
