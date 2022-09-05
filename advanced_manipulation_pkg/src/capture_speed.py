#! /usr/bin/env python
import sys
import os
import rospy
import time
import moveit_commander
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import math
from os import listdir
from os.path import isfile, join, abspath


path = os.path.join(os.path.dirname(__file__), '../files')
path_file = ""
record = False

#ROS init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Capture_speed', anonymous=True)

def my_subscriber_cb(info):
        global path
        global path_file
        global record
        files_list = []
        if info.data and not record:
                try:
                        files_list = os.listdir(path)
                        index_list = []
                        for f in files_list:
                                index_list.append(int(f.split("_")[0]))
                except:
                        index_list = [0] 
                if len(files_list) == 0:
                        index_list = [0] 
                path_file = path + "/" + str(max(index_list) + 1) + "_" + "speed.txt"
                file_record = open(path_file,'w')
                file_record.close()
        record = info.data

my_subscriber = rospy.Subscriber('/record_demo', Bool, my_subscriber_cb)


#Defines the movegroups
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_left = moveit_commander.MoveGroupCommander("arm_left")
arm_right = moveit_commander.MoveGroupCommander("arm_right") #ATC
arms = moveit_commander.MoveGroupCommander("arms")
torso = moveit_commander.MoveGroupCommander("torso")

arm_left.clear_pose_targets()
arm_right.clear_pose_targets()
arms.clear_pose_targets()
torso.clear_pose_targets()

trash_pose = arm_left.get_current_pose()


def compute_distance(pose1, pose2):
        dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)
        return dist


class MainWindow(QtWidgets.QMainWindow):
        """
        Class of the window with the graph for plotting the tactile data
        """

        def __init__(self, *args, **kwargs):
                """
                Constructor. Graph initialization and stablishes the refresh rate
                """
                global tactile_data
                super(MainWindow, self).__init__(*args, **kwargs)

                self.graphWidget = pg.PlotWidget()
                self.setCentralWidget(self.graphWidget)
                self.graphWidget.setTitle("EE linear velocity (mm/s)")

                self.x = list(range(100))  # 100 time points
                self.y = [0 for _ in range(100)]  # 100 data points
                pose_init = Pose()
                self.poses = [pose_init]

                self.graphWidget.setBackground('w')

                pen = pg.mkPen(color=(255, 0, 0))
                self.data_line =  self.graphWidget.plot(self.x, self.y, pen=pen)

                self.last_time = time.time()

                self.timer = QtCore.QTimer()
                self.timer.setInterval(500) #Every 50ms calls the following funtion
                self.timer.timeout.connect(self.update_plot_data)
                self.timer.start()

        def update_plot_data(self):

                pose_left = arm_left.get_current_pose().pose
                self.poses.append(pose_left)
                time_diff = time.time() - self.last_time
                self.last_time = time.time()
                speed = int((compute_distance(self.poses[-1], self.poses[-2])*1000)/(time_diff))

                self.x = self.x[1:]  # Remove the first y element.
                self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.

                self.y = self.y[1:]  # Remove the first
                #self.y.append(int(pose_left.position.x * 1000))  # Add a new random value.
                self.y.append(speed)  # Add a new random value.

                self.data_line.setData(self.x, self.y)  # Update the data.

                if record:
                        file = open(path_file,'a')
                        file.write(str(speed) + ";" + str(self.poses[-1].position.x) + ";" + str(self.poses[-1].position.y) + "\n")
                        file.close()


if __name__ == "__main__":
	app = QtWidgets.QApplication(sys.argv)
	w = MainWindow() #Initialization of the class of the graph window
	w.show()
	sys.exit(app.exec_())