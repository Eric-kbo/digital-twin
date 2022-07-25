from panda3d.core import Vec3,Vec4

from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
base = ShowBase()

base.cam.setPos(0, -2, 0.4)
base.cam.lookAt(0, 0, 0.4)

import simplepbr
pipeline = simplepbr.init()
pipeline.enable_shadows=True

from panda3d.core import AmbientLight

alight = AmbientLight('ambientLight')
alight.set_color(Vec4(0.4, 0.4, 0.4, 1))
alight_np = base.render.attach_new_node(alight)
base.render.setLight(alight_np)

from panda3d.core import DirectionalLight

dlight = DirectionalLight('directionalLight')
dlight.set_color(Vec4(0.7, 0.7, 0.7, 1))
dlight.setShadowCaster(True,1024,1024)
lens = dlight.getLens()
lens.setFilmSize(3,3)

dlight_np = base.render.attach_new_node(dlight)
dlight_np.setPos(0,0,10)
dlight_np.lookAt(0,0,0)
base.render.setLight(dlight_np)

def lighting(task):
    import math

    x,y,z = dlight_np.getPos()
    a = task.time / 5
    r = 2
    x = math.cos(a) * r
    y = math.sin(a) * r
    dlight_np.setPos(x,y,z)
    dlight_np.lookAt(0,0,0)
    return task.cont
    
base.taskMgr.add(lighting,'lighting')

from panda3d.bullet import BulletWorld,BulletDebugNode
bullet_world = BulletWorld()
bullet_world.setGravity(Vec3(0, 0, -9.81))

def do_physics(task):
    bullet_world.doPhysics(globalClock.get_dt(),10, 1/180)
    return task.cont
base.task_mgr.add(do_physics, 'package')

debug = BulletDebugNode('Debug')
debug.showWireframe(True)
debug.showConstraints(False)
debug.showBoundingBoxes(False)
debug.showNormals(False)
debug_np = base.render.attachNewNode(debug)
debug_np.show()
bullet_world.setDebugNode(debug)

from models.floor.floor import Floor
floor_np = Floor(base).make(bullet_world)
floor_np.setPos(0,0,-0.001)

from models.robot.robot import Robot
robot = Robot(base,bullet_world)
robot_np = robot.node_path()
robot_np.set_pos(-0.522699,0.261616,0)
bullet_world.doPhysics(globalClock.get_dt(),10, 1/180)

from models.box.box import Box
box_np = Box(base).make(bullet_world)
box_np.setPos(-0.649572,-0.348722,0)

from models.converyor.converyor import Converyor
converyor_np = Converyor(base).make(bullet_world)

from models.camera.camera import Camera
camera_np = Camera(base).make()
camera_np.setPos(0.497927,0,0)

cube_np = base.loader.loadModel("models/cube/cube.bam")
cube_np.set_depth_offset(-1)

from copy import deepcopy
model = deepcopy(cube_np)
model.reparentTo(base.render)
package_np = model.find('**/Cube/+BulletRigidBodyNode')
package_np.setPos(0.0200741, 0.26172, 0.521254)
package_np.setHpr(0,0,180)
package = package_np.node()
bullet_world.attachRigidBody(package)

packages = list()
def package(task):
    import random

    # model = deepcopy(cube_np)
    # model.reparentTo(base.render)
    # package_np = model.find('**/Cube/+BulletRigidBodyNode')
    # package_np.setPos(random.uniform(-0.14,0.16),-0.7,0.5)
    # packages.append(package_np)

    # package = package_np.node()
    # package.apply_central_impulse(Vec3(0,0.001 * 0.051,0))
    # bullet_world.attachRigidBody(package)

    return task.again

base.task_mgr.do_method_later(1,package, 'package')

base.accept('1',lambda: robot.joint1_degrees(robot.joint1.get_hinge_angle() + -5))
base.accept('q',lambda: robot.joint1_degrees(robot.joint1.get_hinge_angle() + 5))
base.accept('2',lambda: robot.joint2_degrees(robot.joint2.get_hinge_angle() + -5))
base.accept('w',lambda: robot.joint2_degrees(robot.joint2.get_hinge_angle() + 5))
base.accept('3',lambda: robot.joint3_degrees(robot.joint3.get_hinge_angle() + -5))
base.accept('e',lambda: robot.joint3_degrees(robot.joint3.get_hinge_angle() + 5))
base.accept('4',lambda: robot.joint4_degrees(robot.joint4.get_hinge_angle() + -5))
base.accept('r',lambda: robot.joint4_degrees(robot.joint4.get_hinge_angle() + 5))
base.accept('5',lambda: robot.joint5_degrees(robot.joint5.get_hinge_angle() + -5))
base.accept('t',lambda: robot.joint5_degrees(robot.joint5.get_hinge_angle() + 5))
base.accept('6',lambda: robot.joint6_degrees(robot.joint6.get_hinge_angle() + -5))
base.accept('y',lambda: robot.joint6_degrees(robot.joint6.get_hinge_angle() + 5))
base.accept('7',lambda: robot.joint7_degrees(robot.joint7.get_hinge_angle() + -5))
base.accept('u',lambda: robot.joint7_degrees(robot.joint7.get_hinge_angle() + 5))
base.accept('g',lambda: robot.joint_fingers(True))
base.accept('b',lambda: robot.joint_fingers(False))
base.accept('a',lambda: robot.move(package_np.get_pos()))



base.run()