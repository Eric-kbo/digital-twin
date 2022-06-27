from inspect import stack
from random import random
from xml.etree.ElementTree import QName, tostring
from cv2 import INTER_MAX
from direct.showbase.InputStateGlobal import inputState
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import *
from panda3d.bullet import *
import simplepbr

class Application(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)        
        pipeline = simplepbr.init()
        pipeline.enable_shadows=True

        light = DirectionalLight('dlight')
        light.setColor((1, 1, 1, 1))

        self.lnp = self.render.attachNewNode(light)
        self.lnp.setPos(0,-5,5)
        self.lnp.lookAt(0,0,0)
        self.render.setLight(self.lnp)
        # self.render.setShaderAuto()

        alight = AmbientLight('alight')
        alight.setColor((0.2, 0.2, 0.2, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)
               
        self.debugNP = self.render.attachNewNode(BulletDebugNode('Debug'))
        self.debugNP.show()
        self.debugNP.node().showWireframe(True)
        self.debugNP.node().showConstraints(True)
        self.debugNP.node().showBoundingBoxes(False)
        self.debugNP.node().showNormals(True)

        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -9.81))
        self.world.setDebugNode(self.debugNP.node())

        model = self.loader.loadModel('floor.bam')
        model.reparentTo(self.render)
        floor = model.find('**/floor/+BulletRigidBodyNode')
        floor.node().setMass(0)
        self.world.attachRigidBody(floor.node())

        model = self.loader.loadModel("converyor.bam")
        model.reparentTo(self.render)
        model.setPos(-0.2,-0.04,0.502/2)
        converyor = model.find('**/converyor/+BulletRigidBodyNode')
        converyor.reparentTo(model)
        converyor.node().setMass(0)
        converyor.node().setFriction(0)
        self.world.attachRigidBody(converyor.node())
        
        model = self.loader.loadModel("box.bam")
        model.reparentTo(self.render)
        box = model.find('**/box/+BulletRigidBodyNode')
        box.setPos(.4,-.2,0)
        box.node().setMass(0)
        self.world.attachRigidBody(box.node())

        self.cube = self.loader.loadModel("cube.bam")
        cube = self.cube.find('**/Cube/+BulletRigidBodyNode')
        cube.setPos(-0.2,-0.8,1)

        self.cube2 = self.loader.loadModel("cube2.bam")
        cube2 = self.cube2.find('**/Cube/+BulletRigidBodyNode')
        
        self.camera3d = self.loader.loadModel("3d_camera.bam")
        self.camera3d.setPos(-.7,-.6,0)
        self.camera3d.reparentTo(self.render)

        model = self.loader.loadModel("box.bam")
        model.reparentTo(self.render)
        box = model.find('**/box/+BulletRigidBodyNode')
        box.setPos(.4,-.2,0)
        box.node().setMass(0)
        self.world.attachRigidBody(box.node())

        model = Actor("panda.bam")
        model.reparentTo(self.render)
        self.panda = model
        self.panda.setPos(0.3,0.4,0)
        self.panda.setHpr(180,0,0)

        self.base = model.controlJoint(None, "modelRoot", "base")
        self.link0 = model.controlJoint(None, "modelRoot", "link0")
        self.link1 = model.controlJoint(None, "modelRoot", "link1")
        self.link2 = model.controlJoint(None, "modelRoot", "link2")
        self.link3 = model.controlJoint(None, "modelRoot", "link3")
        self.link4 = model.controlJoint(None, "modelRoot", "link4")
        self.link5 = model.controlJoint(None, "modelRoot", "link5")
        self.hand = model.controlJoint(None, "modelRoot", "link6")
        self.finger_c = model.exposeJoint(None, "modelRoot", "finger_c")
        self.finger_l = model.exposeJoint(None, "modelRoot", "finger_l")
        self.finger_r = model.exposeJoint(None, "modelRoot", "finger_r")
        
        self.taskMgr.add(self.update, 'update')
        self.taskMgr.add(self.lighting,'lighting')
        self.taskMgr.do_method_later(1,self.package, 'package')
        
        self.accept('t', self.test)
        self.accept('p', self.pick)
        self.accept('r', self.reset)
        self.accept('d', self.drop)

    def test(self):
        if not self.packages: return
        
        model = self.packages[-1]
        self.package_picked = model.find('**/Cube/+BulletRigidBodyNode')

        t = self.finger_c.getTransform().getMat() * self.panda.getTransform().getMat()
        finger_c_pos = TransformState.makeMat(t).getPos()
        self.package_picked_pos =  self.package_picked.getPos()
        self.package_picked_hpr = self.package_picked.getHpr()

        segs = LineSegs("lines")
        segs.setThickness( 2.0 )
        segs.setColor( Vec4(1,1,0,1) )
        segs.moveTo(finger_c_pos)
        segs.drawTo(self.package_picked_pos)

        if 'segsnode' in dir(self): self.render.node().removeChild(self.segsnode)
        self.segsnode = segs.create()
        self.render.attachNewNode(self.segsnode)
    
    
    def drop(self):
        pass

    def reset(self):
        def joint_reset(joint,step,i,lower,upper,task):
            if step.length() < 0.25: 
                return task.done

            hpr=joint.getHpr()
            if hpr[i] < lower or hpr[i] > upper: 
                hpr[i] = lower if hpr[i] < lower else upper
                step /= -1

            if 0 < hpr[i] or hpr[i] < 0: step /= -2
            print(hpr[i])

            joint.setHpr(hpr + step)
            return task.again

        for joint in self.joint_list:
            from copy import deepcopy
            self.taskMgr.add(joint_reset,str(joint),extraArgs=[joint[0],deepcopy(joint[1]),joint[2],joint[3],joint[4]],appendTask=True)

    def pick(self):
        def joint_rotate(joint,step,i,lower,upper,offset,task):
            hpr=joint.getHpr()
            joint.setHpr(hpr + step)
            
            return task.again

        def finished(task):
            self.sort -= 1

        self.joint_list = [
            [self.hand,Vec3(0,0,1),2,-10,10], 
            [self.link5,Vec3(1,0,0),0,-65,180],   
            [self.link4,Vec3(0,0,1),2,-170,170],
            [self.link3,Vec3(1,0,0),0,-180,0,90],
            [self.link2,Vec3(0,0,1),2,-170,170],
            [self.link1,Vec3(1,0,0),0,-170,170,0]
        ]

        self.sort = 2
        self.taskMgr.add(joint_rotate,'joint_rotate',extraArgs=self.joint_list[3],appendTask=True,uponDeath=finished)
        self.taskMgr.add(joint_rotate,'joint_rotate',extraArgs=self.joint_list[5],appendTask=True,uponDeath=finished)
        
    packages = list()
    def package(self,task):
        from copy import deepcopy
        import random

        model = deepcopy(self.cube)
        model.reparentTo(self.render)
        new_package = model.find('**/Cube/+BulletRigidBodyNode')
        new_package.setPos(random.uniform(-0.1,-0.3),-0.8,0.54)
        self.world.attachRigidBody(new_package.node())
        new_package.node().applyCentralForce(Vec3(0,0.1,0))
        self.packages.append(model)

        return task.again

    def lighting(self,task):
        import math

        x,y,z = self.lnp.getPos()
        a = task.time / 5
        x = math.cos(a) * 10
        y = math.sin(a) * 10
        self.lnp.setPos(x,y,z)
        self.lnp.lookAt(0,0,0)
        return task.cont

    def update(self, task):
        self.world.doPhysics(globalClock.getDt(),1 , 1/180)
        return task.cont

app = Application()
app.run()
