from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletWorld

class Camera():
    def __init__(self,base:ShowBase):
        self.collision_np = model = base.loader.loadModel('models/camera/camera.bam')
        model.reparent_to(base.render)
        model.setDepthOffset(-1)
        pass

    def make(self):
        return self.collision_np