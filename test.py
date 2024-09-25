import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from math import pi

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(),  
        rtb.RevoluteMDH(alpha=-pi/2 ,d=0.2 , offset=-pi/2),  
        rtb.RevoluteMDH(a=0.7 ,d=-0.2 ,offset= 3*pi/4),  
        rtb.RevoluteMDH(a=0.7 ,d=0.2 ,offset= -pi/4),
        rtb.RevoluteMDH(alpha=pi/2 ,d=-0.3 ,offset= pi/2),
        rtb.RevoluteMDH(alpha=pi/2),
    ],
    tool = SE3(0,0,0.5),
    name = "6DOF_Robot"
)

print(robot)


#จงหา POSE ของเฟรม {e} เมื่อกำหนดให้ [$q_1$, $q_2$, $q_3$, $q_4$, $q_5$, $q_6$] เท่ากับ 45, 30, 60, 30, 90, และ 270 องศา ตามลำดับ
qe = [pi/4, pi/6, pi/3, pi/6, pi/2, 3*pi/2]

P_0e = robot.fkine(qe)
print(np.deg2rad(qe))
print(P_0e)



#จงแสดงให้เห็นว่าคำตอบข้อที่ตอบมาในข้อที่ 2.1. เป็นคำตอบที่ถูกต้อง
def Modify_XZ(tx,rx,tz,rz):
    return SE3.Rx(rx) * SE3(tx,0,0) * SE3.Rz(rz) * SE3(0,0,tz)
def Modify_XZ(a, alpha, d, theta):
    # X-Z Parameter transformation: Rotation X(alpha), Translation X(a), Rotation Z(theta), Translation Z(d)
    return SE3.Rx(alpha) * SE3(a, 0, 0) * SE3.Rz(theta) * SE3(0, 0, d)

pose = SE3()
set_a = [0, 0, 0.7, 0.7, 0, 0]
set_alpha = [0, -np.pi/2, 0, 0, -np.pi/2, np.pi/2]
set_d = [0, 0.2, -0.2, 0.2, -0.3, 0]
set_offset = [0, -np.pi/2, 3*np.pi/4, -np.pi/4, np.pi/2, 0]

# Iterate over each link
for i in range(len(robot.links)):
    pose = pose * Modify_XZ(set_a[i], set_alpha[i], set_d[i], q[i] + set_offset[i])

# Multiply with the tool frame
pose = pose * robot.tool
print(pose)