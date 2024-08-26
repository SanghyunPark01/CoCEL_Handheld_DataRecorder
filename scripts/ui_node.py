#!/usr/bin/env python3
import sys
import os
import time
from threading import Thread
import webbrowser
from queue import Queue
import copy
import numpy as np
import cv2
import subprocess

import rospy
from std_msgs.msg import *
from CoCEL_Handheld_DataRecorder.srv import *
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from livox_ros_driver.msg import *
from cv_bridge import CvBridge, CvBridgeError

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui


path__ = os.path.dirname(os.path.realpath(__file__))

sys.path.append(path__)
import ui_utility

path__ = path__ + "/"
ui_path = path__ + "frontend.ui"
logo_path = path__ + "logo.png"

q_UI_form = uic.loadUiType(ui_path)[0]

class UI(QMainWindow,q_UI_form):
    def __init__(self):
        # @@@@@@@@@@@@@@@@@@@@@@
        # @@@@@ Initialize @@@@@
        # @@@@@@@@@@@@@@@@@@@@@@
        super().__init__()
        self.setupUi(self)
        cocel_logo = cv2.imread(logo_path, cv2.IMREAD_UNCHANGED)
        # cocel_logo = cv2.resize(cocel_logo, dsize = (0,0), fx = 0.5, fy = 0.5)
        cocel_logo = cv2.resize(cocel_logo, dsize = (0,0), fx = 0.75, fy = 0.75)

        pix_map = ui_utility.covert_cv2qt_rgba(cocel_logo)
        self.label_cocel_logo.setPixmap(pix_map)
        self.pushButton_lab_link.clicked.connect(lambda: webbrowser.open('https://cocel.postech.ac.kr/'))
        self.pushButton_record_stop.setEnabled(False)
        self.radioButton_visualize_off.setChecked(True)
        home_path = os.path.expanduser('~')
        self.lineEdit_save_dir_bag.setText(home_path+"/test")
        self.lineEdit_save_dir_log.setText(home_path+"/log")
        self.radioButton_auto_naming_on.setChecked(True)
        self._m_naming_idx = 0
        self._m_path_name_std = self.lineEdit_save_dir_bag.text()
        self.textBrowser_log.insertPlainText("[INIT]UI Initialized")

        widget_geo = self.widget_vis_lidar.geometry()
        gl_x = self.groupBox_3.geometry().x()+widget_geo.x()
        gl_y = self.groupBox_3.geometry().y()+widget_geo.y()
        gl_w = widget_geo.width()
        gl_h = widget_geo.height()
        self.widget_vis_lidar = gl.GLViewWidget()
        self.widget_vis_lidar.setGeometry(gl_x, gl_y, gl_w, gl_h)
        self.layout().addWidget(self.widget_vis_lidar)
        self._m_scatter_plot = gl.GLScatterPlotItem()
        self.widget_vis_lidar.addItem(self._m_scatter_plot)

        # @@@@@@@@@@@@@@@@@@@@@@
        # @@@@@@@ Set UI @@@@@@@
        # @@@@@@@@@@@@@@@@@@@@@@
        self._m_status_record = False
        self.pushButton_close.clicked.connect(self._shutdown_sys)

        self.pushButton_log_clear.clicked.connect(self._clear_log)
        self.pushButton_log_save.clicked.connect(self._save_log)

        self.pushButton_record_start.clicked.connect(self._start_record)
        self.pushButton_record_stop.clicked.connect(self._stop_record)
        self._m_debugging_path = ""

        self._m_visualize = False
        self.radioButton_visualize_on.toggled.connect(self._toggle_visualize_button)

        self.pushButton_slam_0.clicked.connect(self._run_fastlio2)

        # @@@@@@@@@@@@@@@@@@@@@
        # @@@@@@ For ROS @@@@@@
        # @@@@@@@@@@@@@@@@@@@@@
        rospy.init_node('handheld_forntend_ui', anonymous = True)
        self._m_pub_shutdown = rospy.Publisher('/cocel_handheld/shutdown_flag/8513211',Bool,queue_size=10)
        self._m_client_rosbag_record = rospy.ServiceProxy('/cocel_handheld/record_flag/541635134',record_flag)
        self._m_client_rosbag_save = rospy.ServiceProxy('/cocel_handheld/save_flag/846158586',save_bag_path)

        img_topic_name = rospy.get_param("/sensor/cam_topic")
        lidar_topic_name = rospy.get_param("/sensor/lidar_topic")
        is_compressed = rospy.get_param("/setting/img_is_compressed")
        if is_compressed:
            len__ = len(img_topic_name)
            img_topic_name = img_topic_name[:len__ - 11]
        self._m_sub_img = rospy.Subscriber(img_topic_name, Image, self._callback_img, queue_size=1)
        self._m_sub_lidar = rospy.Subscriber(lidar_topic_name, CustomMsg, self._callback_lidar, queue_size=1)

    # @@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@ Callback @@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@
    def _callback_img(self, msg):
        if not self._m_visualize:
            return
        msg_img = copy.deepcopy(msg)

        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(msg_img)

        img_pixmap = ui_utility.convert_cv2qt(cv_img = cv_img)
        self.label_vis_cam.setPixmap(img_pixmap.scaled(self.label_vis_cam.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        
    def _callback_lidar(self, msg_custom_lidar):
        if not self._m_visualize:
            return
        point_cloud = PointCloud()
        channel_intensity = ChannelFloat32()
        channel_intensity.name = "intensity"
        for point in msg_custom_lidar.points:
            p = Point32()
            p.x = point.x
            p.y = point.y
            p.z = point.z
            point_cloud.points.append(p)

            channel_intensity.values.append(point.reflectivity)
        point_cloud.channels.append(channel_intensity)

        points_ = np.array([[p_.x, p_.y, p_.z] for p_ in point_cloud.points])
        # intensities_ = np.array([p_.channels[0] for p_ in point_cloud.points])

        # colors_ = ui_utility.intensity_to_color(intensities_)
        self._m_scatter_plot.setData(pos=points_, color=(255,255,255,255), size=1)

    # @@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ For UI @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@
    def _shutdown_sys(self):
        QCoreApplication.instance().quit()
        self._m_pub_shutdown.publish(True)
    def _clear_log(self):
        self.textBrowser_log.clear()
    def _save_log(self):
        path_log_save = self.lineEdit_save_dir_log.text()

        path_log_save += ".txt"
        try:
            with open(path_log_save, "w") as file:
                log_text = self.textBrowser_log.toPlainText()
                file.write(log_text)
                self._add_log("[SUCCESS]Save Log Success!")
        except:
            self._add_log("[ERROR]Wrong Saving Log Path")

    def _add_log(self, qs_log_msg):
        self.textBrowser_log.insertPlainText("\n")
        self.textBrowser_log.insertPlainText(qs_log_msg)
        self.textBrowser_log.verticalScrollBar().setValue(self.textBrowser_log.verticalScrollBar().maximum())
    def _start_record(self):
        if self._m_status_record:
            self._add_log("[WARN]Already recording...")
            return
        
        flag_data = ""
        if self.radioButton_imu_select.isChecked():
            flag_data += "1"
        else :
            flag_data += "0"
        if self.radioButton_lidar_select.isChecked():
            flag_data += "1"
        else :
            flag_data += "0"
        if self.radioButton_camera_select.isChecked():
            flag_data += "1"
        else :
            flag_data += "0"
        
        if flag_data == "000":
            self._add_log("[WARN]Select Sensor!")
            return

        req_save = save_bag_pathRequest()
        req_save.path = self.lineEdit_save_dir_bag.text() + ".bag"
        self._m_debugging_path = req_save.path
        res_save = self._m_client_rosbag_save(req_save)

        req_flag = record_flagRequest()
        req_flag.flag = flag_data
        res_flag = self._m_client_rosbag_record(req_flag)

        if not res_flag.status:
            self._add_log("[ERROR]Fail to Sart Recording")
            return
        
        if not res_save.status:
            self._add_log("[ERROR]Fail to Sart Recording. Wrong Saving Bagfile Path")
            return

        self._m_status_record = True
        self.pushButton_record_start.setEnabled(False)
        self.pushButton_record_stop.setEnabled(True)
        self.radioButton_imu_select.setEnabled(False)
        self.radioButton_lidar_select.setEnabled(False)
        self.radioButton_camera_select.setEnabled(False)


        self._add_log("[SUCCESS]Start Recording")
        sensor_list = ["IMU","LiDAR","Camera"]
        i = 0
        for sensor_status in flag_data:
            sensor_idx = sensor_list[i]
            msg = "   - "+sensor_idx
            if sensor_status == "0":
                msg += ": Off"
            else: 
                msg+= ": On"
            self._add_log(msg)
            i += 1
        
    def _stop_record(self):
        if not self._m_status_record:
            self._add_log("[WARN]Not recording now")
            return
        
        req_flag = record_flagRequest()
        req_flag.flag = "000"
        res_flag = self._m_client_rosbag_record(req_flag)
        
        if not res_flag:
            self._add_log("[ERROR]Fail to save Recording")
            return
        self._add_log("[SUCCESS]Save Bag File: " + self._m_debugging_path)

        self._m_status_record = False
        self.pushButton_record_start.setEnabled(True)
        self.pushButton_record_stop.setEnabled(False)
        self.radioButton_imu_select.setEnabled(True)
        self.radioButton_lidar_select.setEnabled(True)
        self.radioButton_camera_select.setEnabled(True)
        if self.radioButton_auto_naming_on.isChecked():
            self._m_naming_idx += 1
            self.lineEdit_save_dir_bag.setText(self._m_path_name_std + str(self._m_naming_idx))
        else:
            self._m_path_name_std = self.lineEdit_save_dir_bag.text()
            

    def _toggle_visualize_button(self):
        if self.radioButton_visualize_on.isChecked():
            self._m_visualize = True
        else:
            self._m_visualize = False

    def _run_fastlio2(self):
        fast_lio__ = path__ + "fast_lio.sh"
        subprocess.call('sh ' + fast_lio__, shell = True)

if __name__ == '__main__':
    try:
        qApp = QApplication(sys.argv)
        qt_ui = UI()

        qt_ui.show()

        qApp.exec_()

    except:
        pass
