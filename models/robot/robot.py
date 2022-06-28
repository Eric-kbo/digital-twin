from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3,Vec4,Point3,BitMask32,TransformState,LineSegs
from panda3d.bullet import BulletWorld,BulletHingeConstraint,BulletSliderConstraint

class Robot():
    def __init__(self,base:ShowBase,bullet_world:BulletWorld):
        self.base = base
        self.bullet_world = bullet_world

        self.link0_np = link0_np = base.loader.loadModel('models/robot/meshes/collision/link0.bam').find('**/+BulletRigidBodyNode')
        link0_np.reparent_to(base.render)
        link0_np.set_collide_mask(BitMask32.bit(0))
        link0_np.set_pos(0,0,0)
        
        link0 = link0_np.node()
        link0.removeAllChildren()
        link0.setMass(0)
        bullet_world.attach_rigid_body(link0)

        model = base.loader.loadModel('models/robot/meshes/visual/link0.bam')
        model.reparent_to(link0_np)
        model.set_depth_offset(-1)

        link1_np = base.loader.loadModel('models/robot/meshes/collision/link1.bam').find('**/+BulletRigidBodyNode')
        link1_np.reparent_to(base.render)
        link1_np.set_collide_mask(BitMask32.bit(1))
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
        hinge.setLimit(-170, 170)
        
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)

        link2_np = base.loader.loadModel('models/robot/meshes/collision/link2.bam').find('**/+BulletRigidBodyNode')
        link2_np.reparent_to(base.render)
        link2_np.setCollideMask(BitMask32.bit(2))
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
        hinge.setLimit(-105, 105)
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)
        self.joint2_target_degrees = hinge.get_hinge_angle()
        self.joint2_accelerate_lately = 0
        
        def joint2_controlling(task):
            from direct.showbase.ShowBaseGlobal import globalClock
            dt = globalClock.get_dt()
            
            offset_degrees = self.joint2_target_degrees - self.joint2.get_hinge_angle()
            
            print(self.joint2.get_hinge_angle(),offset_degrees)
            if offset_degrees != 0:
                target = 2.1750 if offset_degrees > 0 else -2.1750
                self.joint2.set_motor_target(target * 0.1,0.1)
            else:
                pass
                
                
            #     impulse2 = self.joint2.get_applied_impulse() * dt / 1
            #     accelerate = accelerate * dt / 1

            #     print(impulse2,self.joint2.get_applied_impulse())

            #     self.joint2.setMotorTarget(-impulse2,dt)
            #     self.joint2_accelerate_lately = accelerate

            return task.cont

        base.task_mgr.add(joint2_controlling,'joint2_controlling')
            
        link3_np = base.loader.loadModel('models/robot/meshes/collision/link3.bam').find('**/+BulletRigidBodyNode')
        link3_np.reparent_to(base.render)
        link3_np.setCollideMask(BitMask32.bit(3))
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
        hinge.setLimit(-170, 170)
        
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)

        link4_np = base.loader.loadModel('models/robot/meshes/collision/link4.bam').find('**/+BulletRigidBodyNode')
        link4_np.reparent_to(base.render)
        link4_np.setCollideMask(BitMask32.bit(4))
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
        hinge.setLimit(-180, 0)
        
        hinge.enableAngularMotor(True,0,87)
        bullet_world.attachConstraint(hinge)

        link5_np = base.loader.loadModel('models/robot/meshes/collision/link5.bam').find('**/+BulletRigidBodyNode')
        link5_np.reparent_to(base.render)
        link5_np.setCollideMask(BitMask32.bit(5))
        link5_np.setPos(0,0,1)

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
        hinge.setLimit(-170, 170)
        
        hinge.enableAngularMotor(True,0,12)
        bullet_world.attachConstraint(hinge)

        link6_np = base.loader.loadModel('models/robot/meshes/collision/link6.bam').find('**/+BulletRigidBodyNode')
        link6_np.reparent_to(base.render)
        link6_np.setPos(0,0,1.03207)
        link6_np.setCollideMask(BitMask32.bit(6))
        link6_np.setPos(0,0,1)
        
        link6 = link6_np.node()
        link6.removeAllChildren()
        link6.setDeactivationEnabled(False)
        link6.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(link6)
        
        model = base.loader.loadModel('models/robot/meshes/visual/link6.bam')
        model.reparentTo(link6_np)
        model.setDepthOffset(-1)

        pivotA = Point3(0, 0, 0)
        pivotB = Point3(0, 0, 0)
        axisA =  Vec3(0, -1, 0)
        axisB = Vec3(0, 0, 1)

        self.joint6 = hinge = BulletHingeConstraint(link5, link6, pivotA, pivotB, axisA, axisB, True)
        hinge.setLimit(-5, 220)
        
        hinge.enableAngularMotor(True,0,12)
        bullet_world.attachConstraint(hinge)

        link7_np = base.loader.loadModel('models/robot/meshes/collision/link7.bam').find('**/+BulletRigidBodyNode')
        link7_np.reparent_to(base.render)
        link7_np.setPos(0,0,1.03214)
        link7_np.setCollideMask(BitMask32.bit(7))

        link7 = link7_np.node()
        link7.removeAllChildren()
        link7.setDeactivationEnabled(False)
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
        hinge.setLimit(-170, 170)
        
        hinge.enableAngularMotor(True,0,12)
        bullet_world.attachConstraint(hinge)

        # hand
        self.hand_np = hand_np = base.loader.loadModel('models/robot/meshes/collision/hand.bam').find('**/+BulletRigidBodyNode')
        hand_np.reparent_to(base.render)
        hand_np.setPos(0.087827,0,0.924836)
        hand_np.setCollideMask(BitMask32.bit(8))
        
        hand = hand_np.node()
        hand.removeAllChildren()
        hand.setDeactivationEnabled(False)
        hand.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(hand)
        
        model = base.loader.loadModel('models/robot/meshes/visual/hand.bam')
        model.reparentTo(hand_np)
        model.setDepthOffset(-1)

        # joint_hand
        pivotA = Point3(0, 0, 0.051684)
        pivotB = Point3(0, 0, -0.055576)
        axisA =  Vec3(0, 0, 1)
        axisB = Vec3(0, 0, 1)

        self.joint_hand = hinge = BulletHingeConstraint(link7, hand, pivotA, pivotB, axisA, axisB, True)
        hinge.setLimit(0, 0)
        
        bullet_world.attachConstraint(hinge)

        # finger left
        finger_np = base.loader.loadModel('models/robot/meshes/collision/finger.bam').find('**/+BulletRigidBodyNode')
        finger_np.reparent_to(base.render)
        finger_np.setPos(0.087827,0,0.924836)
        finger_np.setHpr(90,0,0)
        finger_np.setCollideMask(BitMask32.bit(9))

        finger = finger_np.node()
        finger.removeAllChildren()
        finger.setDeactivationEnabled(False)
        finger.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(finger)

        model = base.loader.loadModel('models/robot/meshes/visual/finger.bam')
        model.reparentTo(finger_np)
        model.setDepthOffset(-1)

        # joint_finger_left
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

        slider.setTargetLinearMotorVelocity(0)
        slider.setMaxLinearMotorForce(20)
        slider.setPoweredLinearMotor(True)

        # finger right
        finger_np = base.loader.loadModel('models/robot/meshes/collision/finger.bam').find('**/+BulletRigidBodyNode')
        finger_np.reparent_to(base.render)
        finger_np.setPos(0.087827,0,1.924836)
        finger_np.setCollideMask(BitMask32.bit(9))
        
        finger = finger_np.node()
        finger.removeAllChildren()
        finger.setDeactivationEnabled(False)
        finger.setInertia(Vec3(0.1,0.1,0.1))
        bullet_world.attachRigidBody(finger)

        model = base.loader.loadModel('models/robot/meshes/visual/finger.bam')
        model.reparentTo(finger_np)
        model.setDepthOffset(-1)
        
        # joint_finger_right
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
        slider.setTargetLinearMotorVelocity(0)
        slider.setMaxLinearMotorForce(20)
        slider.setPoweredLinearMotor(True)

        base.task_mgr.add(self.update,'line')
        
        self.segs = LineSegs("lines")
        self.segs.moveTo(Vec3(0,0,0))
        self.segs.drawTo(Vec3(0,0,0))
        self.segs.setColor(Vec4(1,1,0,1))
        self.segs.setThickness( 1.0 )

        self.line = self.base.render.attachNewNode(self.segs.create())
        pass

    def node_path(self):
        return self.link0_np

    def update(self,task):
        from direct.showbase.ShowBaseGlobal import globalClock
        dt = globalClock.get_dt()

        # if self.impulse4 + self.joint4.get_applied_impulse() != 0:
        #     self.impulse4 = self.joint4.get_applied_impulse()
        #     self.joint4.setMotorTarget(-self.impulse4 * dt / 1,dt)

        self.bullet_world.doPhysics(dt, 5, 1.0/180.0)

        pick_up = self.hand_np.get_quat().get_up()
        self.pick_pos = self.hand_np.getPos() + pick_up * 0.10
        
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
    def joint2_degrees(self,degress):
        self.joint2_target_degrees = degress
        
    