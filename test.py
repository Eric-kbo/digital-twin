from panda3d.core import Vec3,Vec4

from direct.showbase.ShowBase import ShowBase
base = ShowBase()

base.cam.setPos(0, -2, 0.4)
base.cam.lookAt(0, 0, 0.4)

import simplepbr
# pipeline = simplepbr.init()
# pipeline.enable_shadows=True

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

debug = BulletDebugNode('Debug')
debug_np = base.render.attachNewNode(debug)
debug_np.show()
debug.showWireframe(False)
debug.showConstraints(False)
debug.showBoundingBoxes(False)
debug.showNormals(True)
bullet_world.setDebugNode(debug)

from models.floor.floor import Floor
floor_np = Floor(base).make(bullet_world)

from models.converyor.converyor import Converyor
converyor_np = Converyor(base).make(bullet_world)

from models.camera.camera import Camera
camera_np = Camera(base).make()
camera_np.setPos(0.488625,-0.626974,0)

from models.box.box import Box
box_np = Box(base).make(bullet_world)
box_np.setPos(-0.649572,-0.49388,0)

from models.robot.robot import Robot
robot = Robot(base,bullet_world)
robot_np = robot.node_path()
robot_np.set_pos(-0.529451,0.325756,0)

cube_np = base.loader.loadModel("models/cube/cube.bam")
cube_np.set_depth_offset(-1)

packages = list()
def package(task):
    from copy import deepcopy
    import random

    model = deepcopy(cube_np)
    model.reparentTo(base.render)
    package_np = model.find('**/Cube/+BulletRigidBodyNode')
    package_np.setPos(random.uniform(-0.14,0.16),-0.7,0.5)
    packages.append(package_np)

    package = package_np.node()
    package.apply_central_impulse(Vec3(0,0.001 * 0.051,0))
    bullet_world.attachRigidBody(package)

    robot.pick(packages[0].getPos())

    from direct.showbase.ShowBaseGlobal import globalClock
    dt = globalClock.getDt()
    bullet_world.doPhysics(dt, 5, 1.0/180.0)
    return task.again

base.task_mgr.do_method_later(1,package, 'package')


base.accept('q',lambda: robot.joint2_degrees(-40))
base.accept('e',lambda: robot.joint2_degrees(40))


base.run()