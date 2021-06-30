
import pybullet as p
import time
import pybullet_data
import socket
import math
import threading
# vars

lf_angles = [90, 90, 0]
rf_angles = [90, 90, 0]
lb_angles = [90, 90, 0]
rb_angles = [90, 90, 0]

UDP_IP = "127.0.0.1"
UDP_PORT = 1234


# set up udp
sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))


# Set up phy engine
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

# optionally "C:\Users\YungThunder\Documents\GitHub\RobotDog\Software"
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")


#p.setAdditionalSearchPath("")

p.setGravity(0, 0, -10)

startPos = [0, 0, 0.10]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
dogId = p.loadURDF("spotSimple.urdf", startPos, startOrientation)
# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
# set up cam
p.resetDebugVisualizerCamera(0.4, 50, -35, startPos)


# set motors
p.setJointMotorControl2(dogId, 0, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 1, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 3, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)

# print(val)
p.setJointMotorControl2(dogId, 5, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 6, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 8, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)

# print(val)
p.setJointMotorControl2(dogId, 10, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 11, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 13, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)

# print(val)
p.setJointMotorControl2(dogId, 15, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 16, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)
p.setJointMotorControl2(dogId, 18, controlMode=p.POSITION_CONTROL,
                        force=99, targetPosition=0)

# print(val)


# step phy engine

    # buffer size is 1024 bytes

    #print("received message: %s" % data)
    #
    # print(val)
# Define a function for the thread
def readUDP():
    p.setJointMotorControl2(dogId, 0, controlMode=p.POSITION_CONTROL,
                                force=99, targetPosition=0)


    p.setJointMotorControl2(dogId, 1, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    p.setJointMotorControl2(dogId, 3, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)

    # print(val)
    p.setJointMotorControl2(dogId, 5, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    p.setJointMotorControl2(dogId, 6, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    p.setJointMotorControl2(dogId, 8, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)

    # print(val)
    p.setJointMotorControl2(dogId, 10, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    p.setJointMotorControl2(dogId, 11, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    p.setJointMotorControl2(dogId, 13, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)

    # print(val)
    p.setJointMotorControl2(dogId, 15, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    p.setJointMotorControl2(dogId, 16, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    p.setJointMotorControl2(dogId, 18, controlMode=p.POSITION_CONTROL,
                            force=99, targetPosition=0)
    while True:
        data = sock.recv(1024)
        if (data.decode("utf-8") != "--"):
            print(data)
            junk1, lf_angles[0], lf_angles[1], lf_angles[2], rf_angles[0], rf_angles[1], rf_angles[2], lb_angles[0], lb_angles[
            1], lb_angles[2], rb_angles[0], rb_angles[1], rb_angles[2], junk = data.decode("utf-8").split(' ')
            p.setJointMotorControl2(dogId, 0, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(lf_angles[0])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 1, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(lf_angles[1])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 3, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=int(lf_angles[2])/180 * math.pi)

            # print(val)
            p.setJointMotorControl2(dogId, 5, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(rf_angles[0])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 6, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(rf_angles[1])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 8, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=int(rf_angles[2])/180 * math.pi)

            # print(val)
            p.setJointMotorControl2(dogId, 10, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(rb_angles[0])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 11, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(rb_angles[1])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 13, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=int(rb_angles[2])/180 * math.pi)

            # print(val)
            p.setJointMotorControl2(dogId, 15, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(lb_angles[0])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 16, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=(int(lb_angles[1])+90)/180 * math.pi)
            p.setJointMotorControl2(dogId, 18, controlMode=p.POSITION_CONTROL,
                                    force=99, targetPosition=int(lb_angles[2])/180 * math.pi)


def sim():
    while True:
        p.stepSimulation()
        time.sleep(0.001)


if __name__ == "__main__":
    x = threading.Thread(target=sim)
    y = threading.Thread(target=readUDP)
    x.start()
    y.start()
