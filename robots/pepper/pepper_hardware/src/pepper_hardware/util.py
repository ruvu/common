import time
import rospy

start_time_tic_toc = 0
name_tic_toc = ""


def tic(name):
    global start_time_tic_toc, name_tic_toc
    start_time_tic_toc = time.time()
    name_tic_toc = name


def toc():
    rospy.loginfo("Elapsed time '" + name_tic_toc + "': " + str(time.time() - start_time_tic_toc) + " seconds.")
