import pyforms
import socket
import time
import threading
from PyQt5 import QtCore
import ikpy
from ikpy.utils import geometry
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import ikpy.inverse_kinematics as ik
import numpy as np
import math

from pyforms.basewidget import BaseWidget
from pyforms.controls import ControlText
from pyforms.controls import ControlButton
from pyforms.controls import ControlLabel
from pyforms.controls import ControlCheckBox
from pyforms.controls import ControlSlider

UDP_IP = "10.0.0.88"
UDP_IP2 = "127.0.0.1"
UDP_PORT = 1234
MESSAGE = b"Hello, World!"

# IK end effector position
lf_curretPosition = [0, 0, 0]
rf_curretPosition = [0, 0, 0]
lb_curretPosition = [0, 0, 0]
rb_curretPosition = [0, 0, 0]

rft1 = 90
rft2 = 90
rft3 = 0

lft1 = 90
lft2 = 90
lft3 = 0

rbt1 = 90
rbt2 = 90
rbt3 = 0

lbt1 = 90
lbt2 = 90
lbt3 = 0

# leg
leg = Chain(name='leg', links=[
    OriginLink(),
    URDFLink(
        name="theta3",
        translation_vector=[0, 0, 0],
        orientation=[0, 0, 0],
        #bounds=(-1.5708, 1.5708),
        rotation=[1, 0, 0],
    ),
    URDFLink(
        name="theta1",
        translation_vector=[0, 0, -0],
        orientation=[0, 0, 0],
        #bounds=(-1.5708, 1.5708),
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="theta2",
        translation_vector=[0, 0, -50],
        orientation=[0, 0, 0],
        #bounds=(0, 3.1416),
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="foot",
        translation_vector=[0, 0, -50],
        orientation=[0, 0, 0],
        rotation=[0, 0, 0],
    )
])


# UDP
class Listener(QtCore.QObject):

    message = QtCore.pyqtSignal(str)

    def __init__(self):

        QtCore.QObject.__init__(self)
        # Socket to talk to server
        print('connected!')
        self.running = True

    def loop(self):
        while self.running:
            Message = b'--'
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(Message, (UDP_IP, UDP_PORT))
            #sock.sendto(Message, (UDP_IP2, UDP_PORT))
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
            self.message.emit(str(data, 'utf-8'))
            time.sleep(0.1)


class SimpleExample1(BaseWidget):

    def __init__(self):
        super(SimpleExample1, self).__init__('Quadrupedal Control Software')

        # Definition of the forms fields

        self._resetPositionButton = ControlButton('Reset Position')

        self._rfLabel = ControlLabel('Right Front Limb')

        self._rftheta1 = ControlSlider(
            'Theta1', default=90, minimum=0, maximum=180, changed_event=self.__updateSVals)
        self._rftheta2 = ControlSlider(
            'Theta2', default=90, minimum=0, maximum=180,  changed_event=self.__updateSVals)
        self._rftheta3 = ControlSlider(
            'Theta3', default=0, minimum=0, maximum=180, changed_event=self.__updateSVals)

        self._lfLabel = ControlLabel('Left Front Limb')

        self._lftheta1 = ControlSlider(
            'Theta1', default=90, minimum=0, maximum=180,  changed_event=self.__updateSVals)
        self._lftheta2 = ControlSlider(
            'Theta2', default=90, minimum=0, maximum=180,  changed_event=self.__updateSVals)
        self._lftheta3 = ControlSlider(
            'Theta3', default=0, minimum=0, maximum=180,  changed_event=self.__updateSVals)

        self._rbLabel = ControlLabel('Right Back Limb')

        self._rbtheta1 = ControlSlider(
            ' Theta1', default=90, minimum=0, maximum=180, changed_event=self.__updateSVals)
        self._rbtheta2 = ControlSlider(
            'Theta2', default=90, minimum=0, maximum=180, changed_event=self.__updateSVals)
        self._rbtheta3 = ControlSlider(
            'Theta3', default=0, minimum=0, maximum=180, changed_event=self.__updateSVals)

        self._lbLabel = ControlLabel('Left Back Limb')

        self._lbtheta1 = ControlSlider(
            'Theta1', default=90, minimum=0, maximum=180, changed_event=self.__updateSVals)
        self._lbtheta2 = ControlSlider(
            'Theta2', default=90, minimum=0, maximum=180,  changed_event=self.__updateSVals)
        self._lbtheta3 = ControlSlider(
            'Theta3', default=0, minimum=0, maximum=180,  changed_event=self.__updateSVals)

        self._rfIKLabel = ControlLabel('Right Front Limb')

        self._rfIKx = ControlSlider(
            'X', default=1, minimum=1, maximum=100, changed_event=self.__updateBodyVals)
        self._rfIKy = ControlSlider(
            'Y', default=0, minimum=-50, maximum=50,  changed_event=self.__updateBodyVals)
        self._rfIKz = ControlSlider(
            'Z', default=80, minimum=60, maximum=100, changed_event=self.__updateBodyVals)

        self._lfIKLabel = ControlLabel('Left Front Limb')

        self._lfIKx = ControlSlider(
            'X', default=1, minimum=1, maximum=100,  changed_event=self.__updateBodyVals)
        self._lfIKy = ControlSlider(
            'Y', default=0, minimum=-50, maximum=50,  changed_event=self.__updateBodyVals)
        self._lfIKz = ControlSlider(
            'Z', default=80, minimum=60, maximum=100,  changed_event=self.__updateBodyVals)

        self._rbIKLabel = ControlLabel('Right Back Limb')

        self._rbIKx = ControlSlider(
            'X', default=1, minimum=1, maximum=100, changed_event=self.__updateBodyVals)
        self._rbIKy = ControlSlider(
            'Y', default=0, minimum=-50, maximum=50, changed_event=self.__updateBodyVals)
        self._rbIKz = ControlSlider(
            'Z', default=80, minimum=60, maximum=100, changed_event=self.__updateBodyVals)

        self._lbIKLabel = ControlLabel('Left Back Limb')

        self._lbIKx = ControlSlider(
            'X', default=1, minimum=1, maximum=100, changed_event=self.__updateBodyVals)
        self._lbIKy = ControlSlider(
            'Y', default=0, minimum=-50, maximum=50,  changed_event=self.__updateBodyVals)
        self._lbIKz = ControlSlider(
            'Z', default=80, minimum=60, maximum=100,  changed_event=self.__updateBodyVals)

        # body

        self._bodyTranslationLabel = ControlLabel('Body Translation')

        self._bodyTranslationX = ControlSlider(
            'X', default=25, minimum=0, maximum=50, changed_event=self.__updateBodyVals)
        self._bodyTranslationY = ControlSlider(
            'Y', default=0, minimum=-30, maximum=30, changed_event=self.__updateBodyVals)
        self._bodyTranslationZ = ControlSlider(
            'Z', default=0, minimum=-30, maximum=30, changed_event=self.__updateBodyVals)

        self._bodyRotationLabel = ControlLabel('Body Rotaion')

        self._bodyRotationX = ControlSlider(
            'X', default=0, minimum=-40, maximum=40, changed_event=self.__updateBodyVals)
        self._bodyRotationY = ControlSlider(
            'Y', default=0, minimum=-5, maximum=5,  changed_event=self.__updateBodyVals)
        self._bodyRotationZ = ControlSlider(
            'Z', default=0, minimum=-20, maximum=20, changed_event=self.__updateBodyVals)

        self.formset = [{
            'a:Leg IK': ['_rfIKLabel', '_rfIKx', '_rfIKy', '_rfIKz', '||', '_lfIKLabel', '_lfIKx', '_lfIKy', '_lfIKz', '=', '_rbIKLabel', '_rbIKx', '_rbIKy', '_rbIKz', '||', '_lbIKLabel', '_lbIKx', '_lbIKy', '_lbIKz', '=', '_bodyTranslationLabel', '_bodyTranslationX', '_bodyTranslationY', '_bodyTranslationZ', '_bodyRotationLabel', '_bodyRotationX', '_bodyRotationY', '_bodyRotationZ'],
            'b:Servos': ['_rfLabel', '_rftheta1', '_rftheta2', '_rftheta3', '=', '_lfLabel', '_lftheta1', '_lftheta2', '_lftheta3', '=', '_rbLabel', '_rbtheta1', '_rbtheta2', '_rbtheta3', '=', '_lbLabel', '_lbtheta1', '_lbtheta2', '_lbtheta3']
        }, '_resetPositionButton']

        self.thread = QtCore.QThread()
        self.listener = Listener()

        self.listener.moveToThread(self.thread)

        self.thread.started.connect(self.listener.loop)
        self.listener.message.connect(self.signal_received)

        QtCore.QTimer.singleShot(0, self.thread.start)

    def signal_received(self, message):
        ___thisi__ = 0
        # print()

        #self.text_edit.append("%s\n" % message)

    def __updateSVals(self):
        rft1 = self._rftheta1.value
        rft2 = self._rftheta2.value
        rft3 = self._rftheta3.value

        lft1 = self._lftheta1.value
        lft2 = self._lftheta2.value
        lft3 = self._lftheta3.value

        rbt1 = self._rbtheta1.value
        rbt2 = self._rbtheta2.value
        rbt3 = self._rbtheta3.value

        lbt1 = self._lbtheta1.value
        lbt2 = self._lbtheta2.value
        lbt3 = self._lbtheta3.value

        Message = bytes(' '+str(rft1) + ' ' + str(rft2) + ' ' + str(rft3) + ' ' + str(lft1) + ' ' + str(lft2) + ' ' + str(
            lft3) + ' ' + str(rbt1) + ' ' + str(rbt2) + ' ' + str(rbt3) + ' ' + str(lbt1) + ' ' + str(lbt2) + ' ' + str(lbt3) + ' '+'\0', 'utf-8')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(Message, (UDP_IP, UDP_PORT))
        sock.sendto(Message, (UDP_IP2, UDP_PORT))

    def __updateBodyVals(self):

        bodyOffset = [100, 50, 0]
        rf_curretPosition = [0, 0, 0]
        lf_curretPosition = [0, 0, 0]
        rb_curretPosition = [0, 0, 0]
        rb_curretPosition = [0, 0, 0]
        rf_oldPosition = [0, 0, 0]
        lf_oldPosition = [0, 0, 0]
        rb_oldPosition = [0, 0, 0]
        lb_oldPosition = [0, 0, 0]

        rf_bodyTranslation = [-self._bodyTranslationX.value + bodyOffset[0],
                              self._bodyTranslationY.value+bodyOffset[1], self._bodyTranslationZ.value + bodyOffset[2]]

        lf_bodyTranslation = [-self._bodyTranslationX.value + bodyOffset[0],
                              self._bodyTranslationY.value-bodyOffset[1], self._bodyTranslationZ.value + bodyOffset[2]]

        rb_bodyTranslation = [-self._bodyTranslationX.value - bodyOffset[0],
                              self._bodyTranslationY.value+bodyOffset[1], self._bodyTranslationZ.value + bodyOffset[2]]

        lb_bodyTranslation = [-self._bodyTranslationX.value - bodyOffset[0],
                              self._bodyTranslationY.value-bodyOffset[1], self._bodyTranslationZ.value + bodyOffset[2]]

        bodyRotation = [self._bodyRotationX.value/180 * math.pi,
                        self._bodyRotationY.value/180 * math.pi, self._bodyRotationZ.value/180 * math.pi]

        bodyRotationMarix = geometry.rpy_matrix(
            bodyRotation[0], bodyRotation[1], bodyRotation[2])

        rf_M1 = np.array([self._rfIKx.value-rf_bodyTranslation[0],
                          self._rfIKy.value + rf_bodyTranslation[1], self._rfIKz.value + rf_bodyTranslation[2]])

        rf_bodyTranslation = list(map(int, rf_M1.dot(bodyRotationMarix)))

        lf_M1 = np.array([self._lfIKx.value-lf_bodyTranslation[0],
                          self._lfIKy.value + lf_bodyTranslation[1], self._lfIKz.value + lf_bodyTranslation[2]])
        lf_bodyTranslation = list(map(int, lf_M1.dot(bodyRotationMarix)))

        rb_M1 = np.array([self._rbIKx.value-rb_bodyTranslation[0],
                          self._rbIKy.value + rb_bodyTranslation[1], self._rbIKz.value + rb_bodyTranslation[2]])
        rb_bodyTranslation = list(map(int, rb_M1.dot(bodyRotationMarix)))

        lb_M1 = np.array([self._lbIKx.value-lb_bodyTranslation[0],
                          self._lbIKy.value+lb_bodyTranslation[1], self._lbIKz.value+lb_bodyTranslation[2]])
        lb_bodyTranslation = list(map(int, lb_M1.dot(bodyRotationMarix)))

        rf_curretPosition = [-self._rfIKx.value-rf_bodyTranslation[0] - bodyOffset[0],
                             self._rfIKy.value+rf_bodyTranslation[1] - bodyOffset[1],  -bodyOffset[2]-(rf_bodyTranslation[2])]

        lf_curretPosition = [-self._lfIKx.value-lf_bodyTranslation[0] - bodyOffset[0],
                             self._lfIKy.value+lf_bodyTranslation[1] + bodyOffset[1], - bodyOffset[2]-(lf_bodyTranslation[2])]

        rb_curretPosition = [-self._rbIKx.value-rb_bodyTranslation[0] + bodyOffset[0],
                             self._rbIKy.value+rb_bodyTranslation[1] - bodyOffset[1], -(rb_bodyTranslation[2]) - bodyOffset[2]]

        lb_curretPosition = [-self._lbIKx.value-lb_bodyTranslation[0] + bodyOffset[0],
                             self._lbIKy.value+lb_bodyTranslation[1] + bodyOffset[1], -(lb_bodyTranslation[2]) - bodyOffset[2]]

        # print(rf_curretPosition)
        if (rf_oldPosition != rf_curretPosition):
            val_rf = ik.inverse_kinematic_optimization(
            leg, geometry.homogeneous_translation_matrix(rf_curretPosition[0], rf_curretPosition[1], rf_curretPosition[2]), np.array(
            [0, rf_oldPosition[0], rf_oldPosition[1], rf_oldPosition[2], 0]), regularization_parameter=None, max_iter=12, orientation_mode=None, no_position=False)

        if (lf_oldPosition != lf_curretPosition):
            val_lf = ik.inverse_kinematic_optimization(
                leg, geometry.homogeneous_translation_matrix(lf_curretPosition[0], lf_curretPosition[1], lf_curretPosition[2]), np.array(
                    [0, lf_oldPosition[0], lf_oldPosition[1], lf_oldPosition[2], 0]), regularization_parameter=None, max_iter=12, orientation_mode=None, no_position=False)

        if (rb_oldPosition != rb_curretPosition):
            val_rb = ik.inverse_kinematic_optimization(
                leg, geometry.homogeneous_translation_matrix(rb_curretPosition[0], rb_curretPosition[1], rb_curretPosition[2]), np.array(
                    [0, rb_oldPosition[0], rb_oldPosition[1], rb_oldPosition[2], 0]), regularization_parameter=None, max_iter=12, orientation_mode=None, no_position=False)

        if (lb_oldPosition != lb_curretPosition):
            val_lb= ik.inverse_kinematic_optimization(
                leg, geometry.homogeneous_translation_matrix(lb_curretPosition[0], lb_curretPosition[1], lb_curretPosition[2]), np.array(
                    [0, lb_oldPosition[0], lb_oldPosition[1], lb_oldPosition[2], 0]), regularization_parameter=None, max_iter=12, orientation_mode=None, no_position=False)

           

        rf_oldPosition = rf_curretPosition
        lf_oldPosition = lf_curretPosition
        rb_oldPosition = rb_curretPosition
        lb_oldPosition = lb_curretPosition

        rft1 = int(val_rf[1] / math.pi * 180) + 90
        rft2 = int(val_rf[2] / math.pi * 180) + 90
        rft3 = int(val_rf[3] / math.pi * 180)

        lft1 = int(val_lf[1] / math.pi * 180) + 90
        lft2 = int(val_lf[2] / math.pi * 180) + 90
        lft3 = int(val_lf[3] / math.pi * 180)

        rbt1 = int(val_rb[1] / math.pi * 180) + 90
        rbt2 = int(val_rb[2] / math.pi * 180) + 90
        rbt3 = int(val_rb[3] / math.pi * 180)

        lbt1 = int(val_lb[1] / math.pi * 180) + 90
        lbt2 = int(val_lb[2] / math.pi * 180) + 90
        lbt3 = int(val_lb[3] / math.pi * 180)

        Message = bytes(' '+str(rft1) + ' ' + str(rft2) + ' ' + str(rft3) + ' ' + str(lft1) + ' ' + str(lft2) + ' ' + str(
            lft3) + ' ' + str(rbt1) + ' ' + str(rbt2) + ' ' + str(rbt3) + ' ' + str(lbt1) + ' ' + str(lbt2) + ' ' + str(lbt3) + ' '+'\0', 'utf-8')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(Message, (UDP_IP, UDP_PORT))
        sock.sendto(Message, (UDP_IP2, UDP_PORT))


# Execute the application
if __name__ == "__main__":
    guiThread = threading.Thread(target=pyforms.start_app(
        SimpleExample1, geometry=(200, 200, 600, 600)))
    guiThread.start()
