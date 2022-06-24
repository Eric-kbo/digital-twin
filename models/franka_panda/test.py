from panda3d.core import *
from panda3d.bullet import *

from direct.showbase.ShowBase import ShowBase
from direct.showbase.InputStateGlobal import inputState

class Game(ShowBase):

  def __init__(self):
    ShowBase.__init__(self)

    self.cam.setPos(0, -2, 0.4)
    self.cam.lookAt(0, 0, 0.4)
    
    # Light
    alight = AmbientLight('ambientLight')
    alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
    alightNP = render.attachNewNode(alight)
    
    dlight = DirectionalLight('directionalLight')
    dlight.setDirection(Vec3(1, 1, -1))
    dlight.setColor(Vec4(0.7, 0.7, 0.7, 1))
    dlightNP = render.attachNewNode(dlight)

    render.clearLight()
    render.setLight(alightNP)
    render.setLight(dlightNP)

    # Input
    self.accept('escape', self.doExit)
    self.accept('r', self.doReset)
    self.accept('f1', self.toggleWireframe)
    self.accept('f2', self.toggleTexture)
    self.accept('f3', self.toggleDebug)
    self.accept('f5', self.doScreenshot)

    self.accept('enter', self.doShoot)

    # Task
    taskMgr.add(self.update, 'updateWorld')

    # Physics
    self.setup()

  # _____HANDLER_____

  def doExit(self):
    self.cleanup()
    sys.exit(1)

  def doReset(self):
    self.cleanup()
    self.setup()

  def toggleWireframe(self):
    self.toggleWireframe()

  def toggleTexture(self):
    self.toggleTexture()

  def toggleDebug(self):
    if self.debugNP.isHidden():
      self.debugNP.show()
    else:
      self.debugNP.hide()

  def doScreenshot(self):
    self.screenshot('Bullet')

  def doShoot(self):
    # Get from/to points from mouse click
    pMouse = self.mouseWatcherNode.getMouse()
    pFrom = Point3()
    pTo = Point3()
    self.camLens.extrude(pMouse, pFrom, pTo)

    pFrom = render.getRelativePoint(self.cam, pFrom)
    pTo = render.getRelativePoint(self.cam, pTo)

    # Calculate initial velocity
    v = pTo - pFrom
    v.normalize()
    v *= 50.0

    # Create bullet
    shape = BulletSphereShape(0.3)
    body = BulletRigidBodyNode('Bullet')
    bodyNP = self.worldNP.attachNewNode(body)
    bodyNP.node().addShape(shape)
    bodyNP.node().setMass(1.0)
    bodyNP.node().setLinearVelocity(v)
    bodyNP.node().setCcdMotionThreshold(1e-7);
    bodyNP.node().setCcdSweptSphereRadius(0.50);
    bodyNP.setCollideMask(BitMask32.allOn())
    bodyNP.setPos(pFrom)

    visNP = loader.loadModel('models/ball.egg')
    visNP.setScale(0.8)
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(bodyNP.node())

    # Remove the bullet again after 2 seconds
    taskMgr.doMethodLater(2, self.doRemove, 'doRemove',
      extraArgs=[bodyNP],
      appendTask=True)

  def doRemove(self, bodyNP, task):
    self.world.removeRigidBody(bodyNP.node())
    bodyNP.removeNode()
    return task.done

  # ____TASK___

  def cleanup(self):
    self.worldNP.removeNode()
    self.worldNP = None
    self.world = None

  def setup(self):
    self.worldNP = render.attachNewNode('World')

    # World
    self.debugNP = self.worldNP.attachNewNode(BulletDebugNode('Debug'))
    self.debugNP.show()
    self.debugNP.node().showWireframe(True)
    self.debugNP.node().showConstraints(True)
    self.debugNP.node().showBoundingBoxes(False)
    self.debugNP.node().showNormals(False)

    self.world = BulletWorld()
    self.world.setGravity(Vec3(0, 0, -9.81))
    self.world.setDebugNode(self.debugNP.node())

    # link0
    bodyA = loader.loadModel('meshes/collision/link0.bam').find('**/+BulletRigidBodyNode').node()
    bodyA.removeAllChildren()
    bodyA.setMass(0)
    bodyNP = self.worldNP.attachNewNode(bodyA)
    bodyNP.setPos(0, 0, 0)
    bodyNP.setCollideMask(BitMask32.bit(0))

    visNP = loader.loadModel('meshes/visual/link0.bam')
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(bodyA)

    # link1
    self.link1 = link1 = loader.loadModel('meshes/collision/link1.bam').find('**/+BulletRigidBodyNode').node()
    link1.removeAllChildren()
    link1.setDeactivationEnabled(False)
    link1.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(link1)
    bodyNP.setCollideMask(BitMask32.bit(1))

    visNP = loader.loadModel('meshes/visual/link1.bam')
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(link1)

    # joint1
    pivotA = Point3(0, 0, 0.14)
    pivotB = Point3(0, 0, -0.193)
    axisA = Vec3(0, 0, 1)
    axisB = Vec3(0, 0, 1)

    self.joint1 = hinge = BulletHingeConstraint(bodyA, link1, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(-170, 170)
    hinge.enableAngularMotor(True,0,87)
    self.world.attachConstraint(hinge)

    # link2
    self.link2 = link2 = loader.loadModel('meshes/collision/link2.bam').find('**/+BulletRigidBodyNode').node()
    link2.removeAllChildren()    
    link2.setDeactivationEnabled(False)
    link2.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(link2)
    bodyNP.setCollideMask(BitMask32.bit(2))
    
    visNP = loader.loadModel('meshes/visual/link2.bam')
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(link2)

    # joint2
    pivotA =  Point3(0, 0, 0)
    pivotB = Point3(0, 0, 0)
    axisA = Vec3(0,1,0)
    axisB = Vec3(0,0,1)

    self.joint2 = hinge = BulletHingeConstraint(link1, link2, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(-105, 105)
    hinge.enableAngularMotor(True,0,87)
    self.world.attachConstraint(hinge)

    # link3
    link3 = loader.loadModel('meshes/collision/link3.bam').find('**/+BulletRigidBodyNode').node()
    link3.removeAllChildren()
    link3.setDeactivationEnabled(False)
    link3.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(link3)
    bodyNP.setCollideMask(BitMask32.bit(3))
    
    visNP = loader.loadModel('meshes/visual/link3.bam')
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(link3)

    # joint3
    pivotA = Point3(0, -0.193, 0)
    pivotB = Point3(0, 0, -0.117)
    axisA =  Vec3(0, -1, 0)
    axisB = Vec3(0, 0, 1)

    self.joint3 = hinge = BulletHingeConstraint(link2, link3, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(-170, 170)
    hinge.enableAngularMotor(True,0,87)
    self.world.attachConstraint(hinge)

    # link4
    link4 = loader.loadModel('meshes/collision/link4.bam').find('**/+BulletRigidBodyNode').node()
    link4.removeAllChildren()
    link4.setDeactivationEnabled(False)
    link4.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(link4)
    bodyNP.setCollideMask(BitMask32.bit(4))
    
    visNP = loader.loadModel('meshes/visual/link4.bam')
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(link4)

    # joint4
    pivotA = Point3(0.082726, 0, 0)
    pivotB = Point3(0, 0, 0)
    axisA =  Vec3(0, -1, 0)
    axisB = Vec3(0, 0, 1)

    self.joint4 = hinge = BulletHingeConstraint(link3, link4, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(-180, 0)
    hinge.enableAngularMotor(True,0,87)
    self.world.attachConstraint(hinge)

    # link5
    link5 = loader.loadModel('meshes/collision/link5.bam').find('**/+BulletRigidBodyNode').node()
    link5.removeAllChildren()
    link5.setDeactivationEnabled(False)
    link5.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(link5)
    bodyNP.setCollideMask(BitMask32.bit(5))
    
    visNP = loader.loadModel('meshes/visual/link5.bam')
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(link5)

    # joint5
    pivotA = Point3(-0.082726, 0.12417, 0)
    pivotB = Point3(0, 0, -0.259416)
    axisA =  Vec3(0, 1, 0)
    axisB = Vec3(0, 0, 1)

    self.joint5 = hinge = BulletHingeConstraint(link4, link5, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(-170, 170)
    hinge.enableAngularMotor(True,0,12)
    self.world.attachConstraint(hinge)

    # link6
    link6 = loader.loadModel('meshes/collision/link6.bam').find('**/+BulletRigidBodyNode').node()
    link6.removeAllChildren()
    link6.setDeactivationEnabled(False)
    link6.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(link6)
    bodyNP.setPos(0,0,1.03207)
    bodyNP.setCollideMask(BitMask32.bit(6))
    
    visNP = loader.loadModel('meshes/visual/link6.bam')
    visNP.reparentTo(bodyNP)

    self.world.attachRigidBody(link6)

    # joint6
    pivotA = Point3(0, 0, 0)
    pivotB = Point3(0, 0, 0)
    axisA =  Vec3(0, -1, 0)
    axisB = Vec3(0, 0, 1)

    self.joint6 = hinge = BulletHingeConstraint(link5, link6, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(-5, 220)
    hinge.enableAngularMotor(True,0,12)
    self.world.attachConstraint(hinge)

    # link7
    link7 = loader.loadModel('meshes/collision/link7.bam').find('**/+BulletRigidBodyNode').node()
    link7.removeAllChildren()
    link7.setDeactivationEnabled(False)
    link7.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(link7)
    bodyNP.setPos(0,0,1.03214)
    bodyNP.setCollideMask(BitMask32.bit(7))
    visNP = loader.loadModel('meshes/visual/link7.bam')
    visNP.reparentTo(bodyNP)
    self.world.attachRigidBody(link7)

    # joint7
    pivotA = Point3(0.087827, 0, 0)
    pivotB = Point3(0, 0, 0)
    axisA =  Vec3(0, -1, 0)
    axisB = Vec3(0, 0, 1)

    self.joint7 = hinge = BulletHingeConstraint(link6, link7, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(-170, 170)
    hinge.enableAngularMotor(True,0,12)
    self.world.attachConstraint(hinge)

    # hand
    self.hand = hand = loader.loadModel('meshes/collision/hand.bam').find('**/+BulletRigidBodyNode').node()
    hand.removeAllChildren()
    hand.setDeactivationEnabled(False)
    hand.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(hand)
    bodyNP.setPos(0.087827,0,0.924836)
    bodyNP.setCollideMask(BitMask32.bit(5))
    visNP = loader.loadModel('meshes/visual/hand.bam')
    visNP.reparentTo(bodyNP)
    self.world.attachRigidBody(hand)

    # joint_hand
    pivotA = Point3(0, 0, 0.051684)
    pivotB = Point3(0, 0, -0.055576)
    axisA =  Vec3(0, 0, 1)
    axisB = Vec3(0, 0, 1)

    self.joint_hand = hinge = BulletHingeConstraint(link7, hand, pivotA, pivotB, axisA, axisB, True)
    hinge.setLimit(0, 0)
    self.world.attachConstraint(hinge)

    # finger left
    finger = loader.loadModel('meshes/collision/finger.bam').find('**/+BulletRigidBodyNode').node()
    finger.removeAllChildren()
    finger.setDeactivationEnabled(False)
    finger.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(finger)
    bodyNP.setPos(0.087827,0,0.924836)
    bodyNP.setHpr(90,0,0)
    bodyNP.setCollideMask(BitMask32.bit(9))
    visNP = loader.loadModel('meshes/visual/finger.bam')
    visNP.reparentTo(bodyNP)
    self.world.attachRigidBody(finger)

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
    self.world.attachConstraint(slider)

    slider.setTargetLinearMotorVelocity(0)
    slider.setMaxLinearMotorForce(20)
    slider.setPoweredLinearMotor(True)

    # finger right
    finger = loader.loadModel('meshes/collision/finger.bam').find('**/+BulletRigidBodyNode').node()
    finger.removeAllChildren()
    finger.setDeactivationEnabled(False)
    finger.setInertia(Vec3(0.1,0.1,0.1))
    bodyNP = self.worldNP.attachNewNode(finger)
    bodyNP.setPos(0.087827,0,0.924836)
    bodyNP.setCollideMask(BitMask32.bit(9))
    visNP = loader.loadModel('meshes/visual/finger.bam')
    visNP.reparentTo(bodyNP)
    self.world.attachRigidBody(finger)
    

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
    self.world.attachConstraint(slider)
    slider.setTargetLinearMotorVelocity(0)
    slider.setMaxLinearMotorForce(20)
    slider.setPoweredLinearMotor(True)

    self.accept('q', self.q)
    self.accept('e', self.e)
    self.accept('x', self.x)
  
    self.impulse2 = self.impulse4 = 0
  def update(self, task):
    dt = globalClock.getDt()
    
    if self.impulse2 + self.joint2.get_applied_impulse() != 0:
      self.impulse2 = self.joint2.get_applied_impulse()
      self.joint2.setMotorTarget(-self.impulse2 * dt / 1,dt)

    if self.impulse4 + self.joint4.get_applied_impulse() != 0:
      self.impulse4 = self.joint4.get_applied_impulse()
      self.joint4.setMotorTarget(-self.impulse4 * dt / 1,dt)
    
    self.world.doPhysics(dt, 5, 1.0/180.0)

    return task.cont
  def q(self):
    # self.joint1.enableAngularMotor(True,-2.1750,1)
    self.joint2.enableAngularMotor(True,-2.1750,87)
    # self.joint3.enableAngularMotor(True,-2.1750,1)
    # self.joint4.enableAngularMotor(True,-2.1750,1)
    # self.joint5.enableAngularMotor(True,-2.1750,1)
  def e(self):
    # self.joint1.enableAngularMotor(True,2.1750,1)
    self.joint2.enableAngularMotor(True,2.1750,87)
    # self.joint3.enableAngularMotor(True,2.1750,1)
    # self.joint4.enableAngularMotor(True,2.1750,1)
    # self.joint5.enableAngularMotor(True,2.1750,1)

  def x(self):
    self.joint2.enableAngularMotor(True,0,87)

game = Game()
game.run()

