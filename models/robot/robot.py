import math
from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3,Vec4,Point3,BitMask32,TransformState,LineSegs
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
        link1_np.set_pos(0,0,1)

        link1 = link1_np.node()
        link1.remove_all_children()
        link1.setDeactivationEnabled(False)
        link1.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link1)
        
        model = base.loader.loadModel('models/robot/meshes/visual/link1.bam')
        model.reparentTo(link1_np)
        model.setDepthOffset(-1)

        pivotA = Point3(0, 0, 0.14)
        pivotB = Point3(0, 0, -0.193)
        axisA = Vec3(0, 0, 1)
        axisB = Vec3(0, 0, 1)

        self.joint1 = hinge = BulletHingeConstraint(link0, link1, pivotA, pivotB, axisA, axisB, True)
        self.joint1_limit = (-170, 170)
        hinge.set_limit(0,0)
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)

        self.link2_np = link2_np = base.loader.loadModel('models/robot/meshes/collision/link2.bam').find('**/+BulletRigidBodyNode')
        link2_np.reparent_to(base.render)
        link2_np.setPos(0,0,1)

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
        link3_np.setPos(0,0,1)
        
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
        link4_np.setPos(0,0,1)

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
        link5_np.setPos(0,0,2)

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
        link6_np.setPos(0,0,1.03207)
        link6_np.setPos(0,0,1)
        
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
        link7_np.setPos(0,0,1.03214)

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
        hinge.set_limit(180,180)
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
        finger_np.setPos(0.086827,0,2)
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

        self.joint_finger_left = slider = BulletSliderConstraint(hand, finger, frameA, frameB, True)
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
        finger_np.setPos(0.087827,0,2)
        
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

        self.joint_finger_right = slider = BulletSliderConstraint(hand, finger, frameA, frameB, True)
        slider.setLowerLinearLimit(0)
        slider.setUpperLinearLimit(0.04)
        slider.setLowerAngularLimit(0)
        slider.setUpperAngularLimit(0)
        bullet_world.attachConstraint(slider)

        slider.setTargetLinearMotorVelocity(-0.2)
        slider.setMaxLinearMotorForce(20)
        slider.setPoweredLinearMotor(True)

        base.task_mgr.add(self.update)
        
        self.segs = LineSegs("lines")
        self.segs.moveTo(Vec3(0,0,0))
        self.segs.drawTo(Vec3(0,0,0))
        self.segs.setColor(Vec4(1,1,0,1))
        self.segs.setThickness( 1.0 )

        # target
        self.stuff_np = None
        bodyB = BulletRigidBodyNode('target')
        bodyB.add_shape(BulletSphereShape(0.001))
        bodyB.set_mass(0.1)
        self.target_np = target_np = base.render.attach_new_node(bodyB)
        target_np.set_collide_mask(BitMask32.all_off())
        target_np.set_pos(2, 0, 0)
        bullet_world.attach(bodyB)

        self.line = self.base.render.attachNewNode(self.segs.create())

        # Spherical Constraint
        pivotA = Point3(0, 0, 0.105)
        pivotB = Point3(0, 0, 0)
        bullet_world.attach_constraint(BulletSphericalConstraint(hand, bodyB, pivotA, pivotB))


    def node_path(self):
        return self.link0_np

    def update(self,task):
        pick_up = self.hand_np.get_quat().get_up()
        self.pick_pos = self.hand_np.getPos() + pick_up * 0.105
        
        self.segs.moveTo(self.pick_pos)
        self.segs.drawTo(self.target_pos)
        # print((self.target_pos - self.pick_pos).length())

        self.base.render.node().removeChild(self.line.node())
        self.line = self.base.render.attachNewNode(self.segs.create())
        
        return task.cont
    
    target_pos = Vec3(0,0,0)
    def pick(self,target_pos,up=Vec3(0,0,1)):
        self.target_pos = target_pos
        pass
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