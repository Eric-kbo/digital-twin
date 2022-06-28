from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletWorld

class Converyor():
    def __init__(self,base:ShowBase):
        self.collision_np = collision_np = base.loader.loadModel('models/converyor/collision/converyor.bam').find('**/+BulletRigidBodyNode')
        collision_np.node().removeAllChildren()
        collision_np.reparent_to(base.render)
        collision_np.set_pos(0,0,0.251)
            
        model = base.loader.loadModel('models/converyor/converyor.bam')
        model.reparent_to(collision_np)
        model.setDepthOffset(-1)
        pass

    def make(self,bullet_world:BulletWorld):
        collision = self.collision_np.node()
        collision.setMass(0)
        collision.set_friction(0)
        bullet_world.attach_rigid_body(collision)

        return self.collision_np