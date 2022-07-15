from cmath import cos
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from panda3d.core import Vec3,LMatrix4,deg_2_rad,rad_2_deg

目标 = np.array([0.02,0.261616,0.5])
目标姿态 = Vec3(0,0,0) #HPR

原点 = Vec3(-0.522699,0.261616,0)
关节0偏移 = Vec3(0,0,0.14)
关节1偏移 = Vec3(0,0,0.192006)
关节2偏移 = Vec3(0,0,0.194)
关节3偏移 = Vec3(0.082726,0,0.121018)
关节4偏移 = Vec3(-0.082726,0,0.124001)
关节5偏移 = Vec3(0,0,0.382976)
关节6偏移 = Vec3(0.087827,0,0)
关节7偏移 = Vec3(0,0,-0.10651)
抓手偏移 = Vec3(0,0,-0.105)

关节0角度限制 = (0,0)
关节1角度限制 = [-170,170]
关节1角度 = 0.
关节2角度限制 = (-105,105)
关节2角度 = 0.
关节3角度限制 = (-170,170)
关节3角度 = 0.
关节4角度限制 = (-180,0)
关节4角度 = 0.
关节5角度限制 = (-170,170)
关节5角度 = 目标姿态[2]
关节6角度限制 = (-5,220)
关节6角度 = 目标姿态[1]
关节7角度限制 = (-170,170)
关节7角度 = 目标姿态[0]
活动关节 = 4

graph = plt.figure(0)
view = graph.add_subplot(projection='3d')
plt.show(block=False)
plt.ion()

while True:
    view.cla()

    关节1开始 = 关节0结束 = 原点 + 关节0偏移
    关节1矩阵 = LMatrix4.rotate_mat(关节1角度,Vec3(0,0,1))
    关节2开始 = 关节1结束 = 关节1开始 + 关节1矩阵.xform_point(关节1偏移)
    关节2矩阵 = LMatrix4.rotate_mat(关节2角度,Vec3(0,1,0)) * 关节1矩阵
    关节3开始 = 关节2结束 = 关节2开始 + 关节2矩阵.xform_point(关节2偏移)
    关节3矩阵 = LMatrix4.rotate_mat(关节3角度,Vec3(0,0,1)) * 关节2矩阵
    关节4开始 = 关节3结束 = 关节3开始 + 关节3矩阵.xform_point(关节3偏移)
    关节4矩阵 = LMatrix4.rotate_mat(关节4角度,Vec3(0,-1,0)) * 关节3矩阵
    关节5开始 = 关节4结束 = 关节4开始 + 关节4矩阵.xform_point(关节4偏移)
    关节5矩阵 = LMatrix4.rotate_mat(关节5角度,Vec3(0,0,1)) * 关节4矩阵
    关节6开始 = 关节5结束 = 关节5开始 + 关节5矩阵.xform_point(关节5偏移)
    关节6矩阵 = LMatrix4.rotate_mat(关节6角度,Vec3(0,1,0)) * 关节5矩阵
    关节7开始 = 关节6结束 = 关节6开始 + 关节6矩阵.xform_point(关节6偏移)
    关节7矩阵 = LMatrix4.rotate_mat(关节7角度,Vec3(0,0,1)) * 关节6矩阵
    抓手开始 = 关节7结束 = np.array(关节7开始 + 关节7矩阵.xform_point(关节7偏移))
    抓手矩阵 = LMatrix4.rotate_mat(0,Vec3(0,0,1)) * 关节7矩阵
    抓手结束 = np.array(抓手开始 + 抓手矩阵.xform_point(抓手偏移))
    抓手_目标 = np.array([抓手结束,目标])
    目标距离 = np.linalg.norm(目标 - 抓手结束)

    关节1_目标 = 目标 - 关节1开始
    关节1_抓手 = 抓手结束 - 关节1开始
    关节1_目标 /= np.linalg.norm(关节1_目标)
    关节1_抓手 /= np.linalg.norm(关节1_抓手)
    关节1_抓手[2] = 关节1_目标[2] = 0
    比值 = np.dot(关节1_目标,关节1_抓手) / (np.linalg.norm(关节1_目标) * np.linalg.norm(关节1_抓手))
    弧度 = np.arccos(1 if 比值 > 1 else 比值)
    水平角度1 = rad_2_deg(弧度)
    if 关节1_目标[1] < 关节1_抓手[1]:
        水平角度1 = -水平角度1

    关节2_目标 = 目标 - 关节2开始
    关节2_抓手 = 抓手结束 - 关节2开始
    关节2_目标 /= np.linalg.norm(关节2_目标)
    关节2_抓手 /= np.linalg.norm(关节2_抓手)
    比值 = np.dot(关节2_抓手,关节2_目标) / (np.linalg.norm(关节2_抓手) * np.linalg.norm(关节2_目标))
    弧度 = np.arccos(1 if 比值 > 1 else 比值)
    垂直角度2 = rad_2_deg(弧度)
    if 关节2_目标[2] > 关节2_抓手[2]:
        垂直角度2 = -垂直角度2

    关节4_目标 = 目标 - 关节4开始
    关节4_抓手 = 抓手结束 - 关节4开始
    关节4_目标 /= np.linalg.norm(关节4_目标)
    关节4_抓手 /= np.linalg.norm(关节4_抓手)
    比值 = np.dot(关节4_抓手,关节4_目标) / (np.linalg.norm(关节4_抓手) * np.linalg.norm(关节4_目标))
    弧度 = np.arccos(1 if 比值 > 1 else 比值)
    垂直角度4 = rad_2_deg(弧度)
    if 关节4_目标[2] < 关节4_抓手[2]:
        垂直角度4 = -垂直角度4

    关节1角度 += 水平角度1

    if 活动关节 == 4:
        关节4角度 += 垂直角度4
        活动关节 = 2
    else:
        关节2角度 += 垂直角度2
        活动关节 = 4

    print(水平角度1,垂直角度2,垂直角度4)
    if abs(水平角度1) + abs(垂直角度2) + abs(垂直角度4) < 0.1:
        if 关节1角度限制[0] > 关节1角度 or 关节1角度 > 关节1角度限制[1] or \
            关节2角度限制[0] > 关节2角度 or 关节2角度 > 关节2角度限制[1] or \
            关节4角度限制[0] > 关节4角度 or 关节4角度 > 关节4角度限制[1]:

            print('角度限制!',关节1角度,关节2角度,关节4角度)
            
        if abs(目标距离) > 0.01:
            print('距离不够!',目标距离)
            
        print('finished')

    关节0 = np.array([原点,关节0结束])
    关节1 = np.array([关节1开始,关节1结束])
    关节2 = np.array([关节2开始,关节2结束])
    关节3 = np.array([关节3开始,关节3结束])
    关节4 = np.array([关节4开始,关节4结束])
    关节5 = np.array([关节5开始,关节5结束])
    关节6 = np.array([关节6开始,关节6结束])
    关节7 = np.array([关节7开始,关节7结束])
    抓手 = np.array([抓手开始,抓手结束])
        
    p0 = np.array([-1.3,-1.3,0])
    p1 = np.array([1.3,-1.3,0])
    p2 = np.array([1.3,1.3,0])
    p3 = np.array([-1.3,1.3,0])
    view.plot(
            [p0[0],p1[0],p2[0],p3[0],p0[0]],
            [p0[1],p1[1],p2[1],p3[1],p0[1]],
            [p0[2],p1[2],p2[2],p3[2],p0[2]],
            linewidth=.5,linestyle='-.')

    p0 = np.array([-1.3,-1.3,1.3])
    p1 = np.array([1.3,-1.3,1.3])
    p2 = np.array([1.3,1.3,1.3])
    p3 = np.array([-1.3,1.3,1.3])
    view.plot(
            [p0[0],p1[0],p2[0],p3[0],p0[0]],
            [p0[1],p1[1],p2[1],p3[1],p0[1]],
            [p0[2],p1[2],p2[2],p3[2],p0[2]],
            linewidth=.5,linestyle='-.')

    view.plot(关节0[:,0],关节0[:,1],关节0[:,2])
    view.plot(抓手[:,0],抓手[:,1],抓手[:,2])
    view.plot(关节7[:,0],关节7[:,1],关节7[:,2])
    view.plot(关节6[:,0],关节6[:,1],关节6[:,2])
    view.plot(关节5[:,0],关节5[:,1],关节5[:,2])
    view.plot(关节4[:,0],关节4[:,1],关节4[:,2])
    view.plot(关节3[:,0],关节3[:,1],关节3[:,2])
    view.plot(关节2[:,0],关节2[:,1],关节2[:,2])
    view.plot(关节1[:,0],关节1[:,1],关节1[:,2])
    view.plot(抓手_目标[:,0],抓手_目标[:,1],抓手_目标[:,2],linestyle='dotted')
    view.scatter(*关节2开始)
    view.scatter(*关节4开始)
    view.scatter(*目标)
    view.text(*目标,f'{目标距离}')


    graph.canvas.draw()
    graph.canvas.flush_events()
    # plt.pause(1)

plt.ioff()