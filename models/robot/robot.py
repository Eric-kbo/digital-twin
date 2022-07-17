from logging import exception
import math
from direct.showbase.ShowBase import ShowBase
import numpy as np
from panda3d.core import Vec3,Vec4,Point3,BitMask32,TransformState,LMatrix4,rad_2_deg,Quat
from panda3d.bullet import BulletWorld,BulletHingeConstraint,BulletSliderConstraint,BulletRigidBodyNode,BulletSphereShape,BulletSphericalConstraint,BulletBoxShape

class Robot():
    def __init__(self,base:ShowBase,bullet_world:BulletWorld):
        self.base = base
        self.bullet_world = bullet_world

        self.link0_np = link0_np = base.loader.loadModel('models/robot/meshes/collision/link0.bam').find('**/+BulletRigidBodyNode')
        link0_np.reparent_to(base.render)
        link0_np.set_pos(0,0,0)
        
        link0 = link0_np.node()
        link0.removeAllChildren()
        link0.setMass(0)
        bullet_world.attach_rigid_body(link0)

        model = base.loader.loadModel('models/robot/meshes/visual/link0.bam')
        model.reparent_to(link0_np)
        model.set_depth_offset(-1)

        self.link1_np = link1_np = base.loader.loadModel('models/robot/meshes/collision/link1.bam').find('**/+BulletRigidBodyNode')
        link1_np.reparent_to(base.render)
        link1_np.set_pos(0,0,0.145447)

        link1 = link1_np.node()
        link1.remove_all_children()
        link1.setDeactivationEnabled(False)
        link1.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link1)
        
        model = base.loader.loadModel('models/robot/meshes/visual/link1.bam')
        model.reparentTo(link1_np)
        model.setDepthOffset(-1)

        pivotA = Point3(0, 0, 0.14)
        pivotB = Point3(0, 0, -0.192006)
        axisA = Vec3(0, 0, 1)
        axisB = Vec3(0, 0, 1)

        self.joint1 = hinge = BulletHingeConstraint(link0, link1, pivotA, pivotB, axisA, axisB, True)
        self.joint1_limit = (-170, 170)
        hinge.set_limit(0,0)
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)

        self.link2_np = link2_np = base.loader.loadModel('models/robot/meshes/collision/link2.bam').find('**/+BulletRigidBodyNode')
        link2_np.reparent_to(base.render)
        link2_np.setPos(0,0,0.332006)

        link2 = link2_np.node()
        link2.removeAllChildren()    
        link2.setDeactivationEnabled(False)
        link2.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link2)
        
        model = base.loader.loadModel('models/robot/meshes/visual/link2.bam')
        model.reparentTo(link2_np)
        model.setDepthOffset(-1)

        pivotA =  Point3(0, 0, 0)
        pivotB = Point3(0, 0, 0)
        axisA = Vec3(0,1,0)
        axisB = Vec3(0,0,1)
 
        self.joint2 = hinge = BulletHingeConstraint(link1, link2, pivotA, pivotB, axisA, axisB, True)
        self.joint2_limit = (-105, 105)
        hinge.set_limit(0,0)
        hinge.enableAngularMotor(True,-2.175,87)
        bullet_world.attachConstraint(hinge)

        self.link3_np = link3_np = base.loader.loadModel('models/robot/meshes/collision/link3.bam').find('**/+BulletRigidBodyNode')
        link3_np.reparent_to(base.render)
        link3_np.setPos(0,0,0.526006)
        
        link3 = link3_np.node()
        link3.removeAllChildren()
        link3.setDeactivationEnabled(False)
        link3.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link3)
        
        model = base.loader.loadModel('models/robot/meshes/visual/link3.bam')
        model.reparentTo(link3_np)
        model.setDepthOffset(-1)

        pivotA = Point3(0, -0.193, 0)
        pivotB = Point3(0, 0, -0.117)
        axisA =  Vec3(0, -1, 0)
        axisB = Vec3(0, 0, 1)

        self.joint3 = hinge = BulletHingeConstraint(link2, link3, pivotA, pivotB, axisA, axisB, True)
        self.joint3_limit = (-170, 170)
        hinge.set_limit(0,0)
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)

        self.link4_np = link4_np = base.loader.loadModel('models/robot/meshes/collision/link4.bam').find('**/+BulletRigidBodyNode')
        link4_np.reparent_to(base.render)
        link4_np.setPos(0,0,0.647024)

        link4 = link4_np.node()
        link4.removeAllChildren()
        link4.setDeactivationEnabled(False)
        link4.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link4)
        
        model = base.loader.loadModel('models/robot/meshes/visual/link4.bam')
        model.reparentTo(link4_np)
        model.setDepthOffset(-1)
        
        pivotA = Point3(0.082726, 0, 0)
        pivotB = Point3(0, 0, 0)
        axisA =  Vec3(0, -1, 0)
        axisB = Vec3(0, 0, 1)

        self.joint4 = hinge = BulletHingeConstraint(link3, link4, pivotA, pivotB, axisA, axisB, True)
        self.joint4_limit = (-180, 0)
        hinge.set_limit(0,0)
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)

        self.link5_np = link5_np = base.loader.loadModel('models/robot/meshes/collision/link5.bam').find('**/+BulletRigidBodyNode')
        link5_np.reparent_to(base.render)
        link5_np.setPos(0,0,0.771025)

        link5 = link5_np.node()
        link5.removeAllChildren()
        link5.setDeactivationEnabled(False)
        link5.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link5)

        model = base.loader.loadModel('models/robot/meshes/visual/link5.bam')
        model.reparentTo(link5_np)
        model.setDepthOffset(-1)

        pivotA = Point3(-0.082726, 0.12417, 0)
        pivotB = Point3(0, 0, -0.259416)
        axisA =  Vec3(0, 1, 0)
        axisB = Vec3(0, 0, 1)

        self.joint5 = hinge = BulletHingeConstraint(link4, link5, pivotA, pivotB, axisA, axisB, True)
        self.joint5_limit = (-170, 170)
        hinge.set_limit(0,0)
        hinge.enableAngularMotor(True,0,12)
        bullet_world.attachConstraint(hinge)

        self.link6_np = link6_np = base.loader.loadModel('models/robot/meshes/collision/link6.bam').find('**/+BulletRigidBodyNode')
        link6_np.reparent_to(base.render)
        link6_np.setPos(0,0,1.03002)
        
        link6 = link6_np.node()
        link6.removeAllChildren()
        link6.setDeactivationEnabled(True)
        bullet_world.attachRigidBody(link6)
        
        model = base.loader.loadModel('models/robot/meshes/visual/link6.bam')
        model.reparentTo(link6_np)
        model.setDepthOffset(-1)

        pivotA = Point3(0, 0, 0)
        pivotB = Point3(0, 0, 0)
        axisA =  Vec3(0, -1, 0)
        axisB = Vec3(0, 0, 1)

        self.joint6 = hinge = BulletHingeConstraint(link5, link6, pivotA, pivotB, axisA, axisB, True)
        self.joint6_limit = (-5, 220)
        hinge.set_limit(30,30)
        hinge.enableAngularMotor(True,2.610,12)
        bullet_world.attachConstraint(hinge)

        link7_np = base.loader.loadModel('models/robot/meshes/collision/link7.bam').find('**/+BulletRigidBodyNode')
        link7_np.reparent_to(base.render)
        link7_np.setPos(0.087827,0,1.03081)

        link7 = link7_np.node()
        link7.removeAllChildren()
        link7.setDeactivationEnabled(True)
        link7.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link7)

        model = base.loader.loadModel('models/robot/meshes/visual/link7.bam')
        model.reparentTo(link7_np)
        model.setDepthOffset(-1)

        pivotA = Point3(0.087827, 0, 0)
        pivotB = Point3(0, 0, 0)
        axisA =  Vec3(0, -1, 0)
        axisB = Vec3(0, 0, 1)

        self.joint7 = hinge = BulletHingeConstraint(link6, link7, pivotA, pivotB, axisA, axisB, True)
        self.joint7_limit = (-170, 170)
        hinge.set_limit(0,0)
        hinge.enableAngularMotor(True,0,12)
        bullet_world.attachConstraint(hinge)

        self.hand_np = hand_np = base.loader.loadModel('models/robot/meshes/collision/hand.bam').find('**/+BulletRigidBodyNode')
        hand_np.reparent_to(base.render)
        hand_np.setPos(0.087827,0,0.824836)

        hand = hand_np.node()
        hand.removeAllChildren()
        hand.setDeactivationEnabled(False)
        hand.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(hand)
        
        model = base.loader.loadModel('models/robot/meshes/visual/hand.bam')
        model.reparentTo(hand_np)
        model.setDepthOffset(-1)

        pivotA = Point3(0, 0, 0.051684)
        pivotB = Point3(0, 0, -0.055576)
        axisA =  Vec3(0, 0, 1)
        axisB = Vec3(0, 0, 1)

        self.joint_hand = hinge = BulletHingeConstraint(link7, hand, pivotA, pivotB, axisA, axisB, True)
        hinge.set_limit(0,0)
        bullet_world.attachConstraint(hinge)
       
        self.finger_left_np = finger_np = base.loader.loadModel('models/robot/meshes/collision/finger.bam').find('**/+BulletRigidBodyNode')
        finger_np.reparent_to(base.render)
        finger_np.setPos(0.086827,0,0.5)
        finger_np.setHpr(90,0,0)

        finger = finger_np.node()
        finger.removeAllChildren()
        finger.setDeactivationEnabled(True)
        finger.set_friction(1)
        finger.set_anisotropic_friction(1)
        finger.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(finger)

        model = base.loader.loadModel('models/robot/meshes/visual/finger.bam')
        model.reparentTo(finger_np)
        model.setDepthOffset(-1)

        pivotA = Point3(-0.041273, 0, 0.058431)
        pivotB = Point3(0, 0, 0)
        frameA = TransformState.makePosHpr(pivotA, Vec3(0, 0, 0))
        frameB = TransformState.makePosHpr(pivotB, Vec3(0, 0, 0))

        self.joint_finger_right = slider = BulletSliderConstraint(hand, finger, frameA, frameB, True)
        slider.setLowerLinearLimit(0)
        slider.setUpperLinearLimit(0.04)
        slider.setLowerAngularLimit(0)
        slider.setUpperAngularLimit(0)
        bullet_world.attachConstraint(slider)

        slider.setTargetLinearMotorVelocity(-0.2)
        slider.setMaxLinearMotorForce(20)
        slider.setPoweredLinearMotor(True)

        self.finger_right_np = finger_np = base.loader.loadModel('models/robot/meshes/collision/finger.bam').find('**/+BulletRigidBodyNode')
        finger_np.reparent_to(base.render)
        finger_np.setPos(0.087827,0,0.5)
        
        finger = finger_np.node()
        finger.removeAllChildren()
        finger.setDeactivationEnabled(True)
        finger.set_friction(1)
        finger.set_anisotropic_friction(1)
        finger.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(finger)

        model = base.loader.loadModel('models/robot/meshes/visual/finger.bam')
        model.reparentTo(finger_np)
        model.setDepthOffset(-1)
        
        pivotA = Point3(0.041273, 0, 0.058431)
        pivotB = Point3(0, 0, 0)
        frameA = TransformState.makePosHpr(pivotA, Vec3(180, 0, 0))
        frameB = TransformState.makePosHpr(pivotB, Vec3(0, 0, 0))

        self.joint_finger_left = slider = BulletSliderConstraint(hand, finger, frameA, frameB, True)
        slider.setLowerLinearLimit(0)
        slider.setUpperLinearLimit(0.04)
        slider.setLowerAngularLimit(0)
        slider.setUpperAngularLimit(0)
        bullet_world.attachConstraint(slider)

        slider.setTargetLinearMotorVelocity(-0.2)
        slider.setMaxLinearMotorForce(20)
        slider.setPoweredLinearMotor(True)

        # target
        self.stuff_np = None
        bodyB = BulletRigidBodyNode('target')
        bodyB.add_shape(BulletSphereShape(0.001))
        bodyB.set_mass(0.1)
        self.target_np = target_np = base.render.attach_new_node(bodyB)
        target_np.set_collide_mask(BitMask32.all_off())
        target_np.set_pos(-99, 0, 0)
        bullet_world.attach(bodyB)

        # Spherical Constraint
        pivotA = Point3(0, 0, 0.105)
        pivotB = Point3(0, 0, 0)
        bullet_world.attach_constraint(BulletSphericalConstraint(hand, bodyB, pivotA, pivotB))

    def node_path(self):
        return self.link0_np
    def joint1_degrees(self,degrees):
        volocity = 2.175 if degrees > self.joint1.get_hinge_angle() else -2.175
        self.joint1.set_limit(low=degrees,high=degrees)
        self.joint1.set_motor_target(volocity,1)
    def joint2_degrees(self,degrees):
        volocity = 2.175 if degrees > self.joint2.get_hinge_angle() else -2.175
        self.joint2.set_limit(low=degrees,high=degrees)
        self.joint2.set_motor_target(volocity,1)
    def joint3_degrees(self,degrees):
        volocity = 2.175 if degrees > self.joint3.get_hinge_angle() else -2.175
        self.joint3.set_limit(low=degrees,high=degrees)
        self.joint3.set_motor_target(volocity,1)
    def joint4_degrees(self,degrees):
        volocity = 2.175 if degrees > self.joint4.get_hinge_angle() else -2.175
        self.joint4.set_limit(low=degrees,high=degrees)
        self.joint4.set_motor_target(volocity,1)
    def joint5_degrees(self,degrees):
        volocity = 2.610 if degrees > self.joint5.get_hinge_angle() else -2.610 
        self.joint5.set_limit(low=degrees,high=degrees)
        self.joint5.set_motor_target(volocity,1)
    def joint6_degrees(self,degrees):
        volocity = 2.610 if degrees > self.joint6.get_hinge_angle() else -2.610 
        self.joint6.set_limit(low=degrees,high=degrees)
        self.joint6.set_motor_target(volocity,1)
    def joint7_degrees(self,degrees):
        volocity = 2.610 if degrees > self.joint7.get_hinge_angle() else -2.610 
        self.joint7.set_limit(low=degrees,high=degrees)
        self.joint7.set_motor_target(volocity,1)

    joint_target = None
    def contacting(self,task):
        res = self.bullet_world.contact_test(self.target_np.node())
        if not res.get_num_contacts(): 
            return task.again

        stuff = res.get_contact(0).get_node1()
        res1 = self.bullet_world.contact_test_pair(self.finger_left_np.node(),stuff)
        
        if not res1.get_num_contacts(): 
            return task.again

        res2 = self.bullet_world.contact_test_pair(self.finger_right_np.node(),stuff)
        if not res2.get_num_contacts():
            return task.again
            
        self.stuff_np = self.base.render.find_path_to(stuff)
        pivotA = Point3(0, 0, 0.105) - (self.stuff_np.get_pos() - self.target_np.get_pos())
        pivotB = Point3(0, 0, 0)

        axisA = -self.hand_np.get_quat().get_up()
        axisB = Vec3(0, 0, 1)
        
        self.joint_finger_left.setTargetLinearMotorVelocity(0.0)
        self.joint_finger_right.setTargetLinearMotorVelocity(0.0)
        self.joint_target = BulletHingeConstraint(self.hand_np.node(),stuff, pivotA, pivotB,axisA, axisB, True)
        self.joint_target.setLimit(0,0)
        self.bullet_world.attach_constraint(self.joint_target)
        return task.done

    def joint_fingers(self,grasp=True):
        self.joint_finger_left.set_target_linear_motor_velocity(0.2 if grasp else -0.2)
        self.joint_finger_right.set_target_linear_motor_velocity(0.2 if grasp else -0.2)
        
        self.base.task_mgr.remove('contacting')
        if grasp:
            if not self.joint_target: 
                self.base.task_mgr.do_method_later(0,self.contacting,'contacting')
        else:
            if self.joint_target: 
                self.bullet_world.remove_constraint(self.joint_target)
                self.joint_target = None

    def IK(self,目标,偏航=0,俯仰=0,翻滚=0):
        目标 = np.array(目标)

        原点 = Vec3(0,0,0)
        关节0偏移 = Vec3(0,0,0.145447)
        关节1偏移 = Vec3(0,0,0.205952)
        关节2偏移 = Vec3(0,0,0.207689)
        关节3偏移 = Vec3(0.138436,0,0.132895)
        关节4偏移 = Vec3(-0.138436,0,0.132895)
        关节5偏移 = Vec3(0,0,0.258891)
        关节6偏移 = Vec3(0.088294,0,0)
        关节7偏移 = Vec3(0,0,-0.107012)
        抓手偏移 = Vec3(0,0,-0.107911)


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
        关节5角度 = -俯仰
        关节6角度限制 = (-5,220)
        关节6角度 = -翻滚
        关节7角度限制 = (-170,170)
        关节7角度 = -偏航
        活动关节 = 1

        while True:
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
            关节6矩阵 = LMatrix4.rotate_mat(关节6角度,Vec3(0,-1,0)) * 关节5矩阵
            关节7开始 = 关节6结束 = 关节6开始 + 关节6矩阵.xform_point(关节6偏移)
            关节7矩阵 = LMatrix4.rotate_mat(关节7角度,Vec3(0,0,-1)) * 关节6矩阵
            抓手开始 = 关节7结束 = np.array(关节7开始 + 关节7矩阵.xform_point(关节7偏移))
            抓手矩阵 = 关节7矩阵
            抓手结束 = np.array(抓手开始 + 抓手矩阵.xform_point(抓手偏移))
            抓手姿态 = Quat()
            抓手姿态.set_from_matrix(抓手矩阵)
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

            h,p,r = 抓手姿态.get_hpr()
            

            print('活动关节',活动关节)
            print( h,p,r )
            if 活动关节 == 0:
                活动关节 = 1
            elif 活动关节 == 1:
                关节1角度 += 水平角度1
                活动关节 = 4
            elif 活动关节 == 2:
                关节2角度 += 垂直角度2
                活动关节 = 5
            elif 活动关节 == 3:
                活动关节 = 5
            elif 活动关节 == 4:
                关节4角度 += 垂直角度4
                活动关节 = 2
            elif 活动关节 == 5:
                关节5角度 += p
                活动关节 = 6
            elif 活动关节 == 6:
                关节6角度 += r
                活动关节 = 7
            elif 活动关节 == 7:
                关节7角度 += -h
                活动关节 = 1
            else:
                pass
            
            if abs(水平角度1) + abs(垂直角度2) + abs(垂直角度4) < 0.5:
                if abs(目标距离) > 0.01:
                    raise Exception('超出范围!',目标距离)

                if 关节1角度限制[0] > 关节1角度 or 关节1角度 > 关节1角度限制[1] or \
                    关节2角度限制[0] > 关节2角度 or 关节2角度 > 关节2角度限制[1] or \
                    关节4角度限制[0] > 关节4角度 or 关节4角度 > 关节4角度限制[1] or \
                    关节5角度限制[0] > 关节5角度 or 关节5角度 > 关节5角度限制[1] or \
                    关节6角度限制[0] > 关节6角度 or 关节6角度 > 关节6角度限制[1] or \
                    关节7角度限制[0] > 关节7角度 or 关节7角度 > 关节7角度限制[1]:
                    raise Exception('角度限制!',[0,关节1角度,关节2角度,关节3角度,关节4角度,关节5角度,关节6角度,关节7角度])

                return [0,关节1角度,关节2角度,关节3角度,关节4角度,关节5角度,关节6角度,关节7角度]
        pass

    def move(self,目标位置,翻滚=0,俯仰=0,偏航=0):
        机械臂位置 = self.link0_np.get_pos()
        关节角度 = self.IK(目标位置-机械臂位置,翻滚,俯仰,偏航)

        self.joint1_degrees(关节角度[1])
        self.joint2_degrees(关节角度[2])
        self.joint3_degrees(关节角度[3])
        self.joint4_degrees(关节角度[4])
        self.joint5_degrees(关节角度[5])
        self.joint6_degrees(关节角度[6])
        self.joint7_degrees(关节角度[7])

        pass

    def grasp(self,目标位置,翻滚,俯仰,偏航):
        机械臂位置 = self.link0_np.get_pos()
        关节角度 = self.IK(目标位置-机械臂位置,翻滚,俯仰,偏航)
        self.joint1_degrees(关节角度[1])
        self.joint2_degrees(关节角度[2])
        self.joint3_degrees(关节角度[3])
        self.joint4_degrees(关节角度[4])
        self.joint5_degrees(关节角度[5])
        self.joint6_degrees(关节角度[6])
        self.joint7_degrees(关节角度[7])
        pass