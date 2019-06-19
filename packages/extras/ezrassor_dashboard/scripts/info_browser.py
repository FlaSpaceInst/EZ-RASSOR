import rospy
import rostopic
import std_msgs.msg
import time
from std_msgs.msg import Int16
from sensor_msgs.msg import Image, Imu
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, Qt

class Ui_Form(object):
    def infoCallback(self, data):
        self.infoData = str(data)

    def addToInfo(self, data):
        self.infoBox.append(self.infoData)
    
    def on_timer(self):
        if(self.infoData != None):
            self.infoBox.append(self.infoData)
            self.infoData = None

    def setupUi(self, Form):
        Form.resize(420, 251)
        self.gridLayout_2 = QtWidgets.QGridLayout(Form)
        self.gridLayout = QtWidgets.QGridLayout()
        self.infoBox = QtWidgets.QTextBrowser(Form)
        self.gridLayout.addWidget(self.infoBox, 2, 0, 1, 1)
        self.label = QtWidgets.QLabel(Form)
        self.label.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

        self.infoData = None
        data = rospy.Subscriber('/ez_main_topic', Int16, lambda data : self.infoCallback(data))
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start()

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "Info"))


if __name__ == "__main__":
    import sys

    rospy.init_node('info_browser', anonymous = True)

    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

