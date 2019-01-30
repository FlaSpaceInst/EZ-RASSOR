import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore, QtGui, QtWidgets
from cv_bridge import CvBridge, CvBridgeError

class Ui_Form(object):
    def addCameras(self):
        topicList = dict(rospy.get_published_topics())

        for i in topicList:
            if 'sensor_msgs/Image' == topicList[i]:
                self.cameraSelect.addItem(i)

    def changeCamera(self, text):
        if text != "Select Camera":
            self.imageSelected = text
            rospy.Subscriber(self.imageSelected, Image, lambda data : self.displayCamera(data))

    def displayCamera(self, data):
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

        self.cameraFrame.setPixmap(pixmapResized)

    def setupUi(self, Form):
        #keep a 37 difference and at lowest (350,387)
        Form.resize(350, 387)

        self.gridLayout_2 = QtWidgets.QGridLayout(Form)
        self.gridLayout = QtWidgets.QGridLayout()
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.cameraSelect = QtWidgets.QComboBox(Form)
        self.cameraSelect.addItem("Select Camera")
        self.addCameras()
        self.verticalLayout.addWidget(self.cameraSelect)
        self.cameraFrame = QtWidgets.QLabel(Form)
        self.cameraFrame.setFrameShape(QtWidgets.QFrame.Box)
        
        self.verticalLayout.addWidget(self.cameraFrame)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)

        self.cameraSelect.currentIndexChanged['QString'].connect(self.changeCamera)


if __name__ == "__main__":
    import sys

    rospy.init_node('camera_widget')

    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())