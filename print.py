import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

from pose_xyt_t import pose_xyt_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

true_poses = []
slam_poses = []
odom_poses = []

for event in log:
    if event.channel == "TRUE_POSE":
        msg = pose_xyt_t.decode(event.data)
        true_poses.append(msg)

        print("Message:")
        print("   timestamp    = %s" % str(msg.utime))
        print("   x    = %s" % str(msg.x))
        print("   y    = %s" % str(msg.y))
        print("")

    elif event.channel == "SLAM_POSE":
        msg = pose_xyt_t.decode(event.data)
        slam_poses.append(msg)

        print("   timestamp    = %s" % str(msg.utime))
        print("   x    = %s" % str(msg.x))
        print("   y    = %s" % str(msg.y))
        print("")

    elif event.channel == "ODOMETRY":
        msg = pose_xyt_t.decode(event.data)
        odom_poses.append(msg)


print(len(true_poses))
print(len(slam_poses))
print(len(odom_poses))

pose_errors = []
odom_errors = []
for slam in slam_poses:
    closest_true = min(true_poses, key=lambda x : abs(x.utime - slam.utime))
    closest_odom = min(odom_poses, key=lambda x : abs(x.utime - slam.utime))
    print(closest_true.utime)
    print(slam.utime)
    pose_errors.append((closest_true.x - slam.x) ** 2 + (closest_true.y - slam.y) ** 2)
    odom_errors.append((closest_odom.x - slam.x) ** 2 + (closest_odom.y - slam.y) ** 2)
    print("")

plt.plot(pose_errors)
plt.title("Error in SLAM pose over the course of the run")
plt.xlabel("Sample index")
plt.ylabel("SLAM to true pose distance")
plt.show()
print("Mean: %s" % str(np.mean(pose_errors)))
print("Std dev: %s" % str(np.std(pose_errors)))
print("Max: %s" % str(np.max(pose_errors)))

plt.plot(odom_errors)
plt.title("Difference between SLAM and odometry poses over the course of the run")
plt.xlabel("Sample index")
plt.ylabel("SLAM to odometry pose distance")
plt.show()

print("Last error in true pose: %s" % str(pose_errors[-1]))
# print("Mean: %s" % str(np.mean(pose_errors)))
# print("Std dev: %s" % str(np.std(pose_errors)))
# print("Max: %s" % str(np.max(pose_errors)))
