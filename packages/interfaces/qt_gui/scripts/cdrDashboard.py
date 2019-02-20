import rospy
import rostopic
import cv2
import numpy as np
import std_msgs.msg
import rosgraph_msgs
from std_msgs.msg import Int16
from sensor_msgs.msg import Image, Imu
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore, QtGui, QtWidgets
from cv_bridge import CvBridge, CvBridgeError

class Ui_Form(object):
    def changeCamera1(self, text):
        if "Select Camera" not in text:
            if self.displayed_1 == True:
                self.sub.unregister()
            self.imageSelected = text
            self.sub = rospy.Subscriber(self.imageSelected, Image, lambda data : self.displayCamera(data, 1))
            self.displayed_1 = True;
            self.infoBox.append("<html><b>Frame 1:</b></html> "+ self.imageSelected)

    def changeCamera2(self, text):
        if "Select Camera" not in text:
            if self.displayed_2 == True:
                self.sub_2.unregister()
            self.imageSelected = text
            self.sub_2 = rospy.Subscriber(self.imageSelected, Image, lambda data : self.displayCamera(data, 2))
            self.displayed_2 = True;
            self.infoBox.append("<html><b>Frame 2:</b></html> "+ self.imageSelected)

    def displayCamera(self, data, camera):
        bridge = CvBridge()
        wd = data.width
        ht = data.height
        camWd = self.cameraFrame.width()-2
        camHt = self.cameraFrame.height()-2

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        np_arr = np.fromstring(data.data, np.uint8)
        sz = (data.height, data.width, data.step / data.width)
        image_pre = np.reshape(np_arr, sz)

        image = cv2.resize(image_pre, (wd, ht), interpolation = cv2.INTER_NEAREST)
        qimage = QImage(image.tostring(), wd, ht, 
                QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimage)
        pixmapResized = pixmap.scaled(camWd, camHt, QtCore.Qt.KeepAspectRatio)

        if camera == 1:
            self.cameraFrame.setPixmap(pixmapResized)
        else:
            self.cameraFrame_2.setPixmap(pixmapResized)

    def addCameras(self):
        topicList = dict(rospy.get_published_topics())
        cameraCount = 0
        self.infoBox.append("<html><b>Adding Cameras</b</html>")

        for i in topicList:
            if 'sensor_msgs/Image' in topicList[i]:
                self.cameraSelect.addItem(i)
                self.cameraSelect_2.addItem(i)
                self.infoBox.append("Added " + i)
                cameraCount = cameraCount + 1
        self.infoBox.append("<html><b>"+str(cameraCount)+" cameras succesfully added</b></html>")

    def autoDig(self):
        self.infoBox.append("Auto Dig Function Called")

    def autoDrive(self):
        self.infoBox.append("Auto Drive Function Called")

    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(679, 548)
        self.infoBox = QtWidgets.QTextBrowser(Form)
        self.infoBox.setGeometry(QtCore.QRect(30, 380, 411, 151))
        self.infoBox.setObjectName("infoBox")
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(30, 360, 41, 20))
        self.label.setFrameShape(QtWidgets.QFrame.Box)
        self.label.setObjectName("label")
        self.widget = QtWidgets.QWidget(Form)
        self.widget.setGeometry(QtCore.QRect(10, 0, 661, 361))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setSpacing(8)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.cameraSelect = QtWidgets.QComboBox(self.widget)
        self.cameraSelect.setObjectName("cameraSelect")
        self.cameraSelect.addItem("Select Camera")

        self.verticalLayout_2.addWidget(self.cameraSelect)
        self.cameraFrame = QtWidgets.QLabel(self.widget)
        self.cameraFrame.setFrameShape(QtWidgets.QFrame.Box)
        self.cameraFrame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.cameraFrame.setText("")
        self.cameraFrame.setObjectName("cameraFrame")
        self.verticalLayout_2.addWidget(self.cameraFrame)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.cameraSelect_2 = QtWidgets.QComboBox(self.widget)
        self.cameraSelect_2.setObjectName("cameraSelect_2")
        self.cameraSelect_2.addItem("Select Camera")
        self.addCameras()

        self.verticalLayout.addWidget(self.cameraSelect_2)
        self.cameraFrame_2 = QtWidgets.QLabel(self.widget)
        self.cameraFrame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.cameraFrame_2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.cameraFrame_2.setText("")
        self.cameraFrame_2.setObjectName("cameraFrame_2")
        self.verticalLayout.addWidget(self.cameraFrame_2)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.widget1 = QtWidgets.QWidget(Form)
        self.widget1.setGeometry(QtCore.QRect(480, 400, 161, 141))
        self.widget1.setObjectName("widget1")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget1)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.autoDigButton = QtWidgets.QPushButton(self.widget1)
        self.autoDigButton.setObjectName("autoDigButton")
        self.verticalLayout_3.addWidget(self.autoDigButton)
        self.AutoDriveButton = QtWidgets.QPushButton(self.widget1)
        self.AutoDriveButton.setObjectName("AutoDriveButton")
        self.verticalLayout_3.addWidget(self.AutoDriveButton)

        self.retranslateUi(Form)
        self.cameraSelect_2.currentIndexChanged['QString'].connect(self.changeCamera2)
        self.cameraSelect.currentIndexChanged['QString'].connect(self.changeCamera1)
        self.autoDigButton.clicked.connect(self.autoDig)
        self.AutoDriveButton.clicked.connect(self.autoDrive)
        QtCore.QMetaObject.connectSlotsByName(Form)
        self.displayed_1 = False;
        self.displayed_2 = False;
        rospy.Subscriber('/ez_main_topic', Int16, lambda data : rospy.loginfo(data))

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "Info"))
        self.autoDigButton.setText(_translate("Form", "Auto Dig"))
        self.AutoDriveButton.setText(_translate("Form", "Auto Drive"))


if __name__ == "__main__":
    import sys

    rospy.init_node('dashboard', anonymous = True)

    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

