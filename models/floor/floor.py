from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletWorld

class Floor():
    def __init__(self,base:ShowBase):
        self.floor_np = floor_np = base.loader.loadModel('models/floor/collision/floor.bam').find('**/+BulletRigidBodyNode')
        floor_np.reparent_to(base.render)
        
        model = base.loader.loadModel('models/floor/floor.bam')
        model.reparent_to(floor_np)
        pass

    def make(self,bullet_world:BulletWorld):
        floor = self.floor_np.node()
        floor.setMass(0)
        
        bullet_world.attach_rigid_body(floor)
        return self.floor_np