import pdb
import time

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from gps.agent.ros.cad.agent_cad import AgentCAD
from gps.agent.ros.cad.util import *


class AgentCADExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)

        # self.use_controller('GPS')
        # for _ in range(10):
        #     self.reset(0)
        #     self.reset(1)
        pdb.set_trace()     # for optional setup, not debugging

    def setup(self):
        self.configure_scene()
        self.grasp_prep()

    def configure_scene(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        print 'Resetting piece'
        self.reset_piece()

        print 'Adding objects to planning scene'
        # self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')
        self.add_object('table', position=[0.5,0.,0.42], size=[0.7,1.5,0.03], type='box')

        for name in ('held_piece', 'fixed_piece'):
            pose = self.get_pose(name)
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=(0.05,0.05,0.0254),
                    filename=self._hyperparams['cad_path'])

    def reset_piece(self):
        quat = Quaternion(*quaternion_from_euler(1.57971, 0.002477, 3.11933))
        pose = Pose(Point(0.841529, 0.209424, 0.501394), quat)
        self.set_pose('held_piece', pose)

    def grasp_prep(self):
        self.use_controller('MoveIt')
        self.ungrip(None)

        target_position = listify(self.get_pose('held_piece').position)
        target_position[0] -= 0.3
        target_position[2] += 0.01
        target_pose = [0,0,0]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

        self.ungrip(15)

        target_position[0] += 0.1
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

    def grasp(self):
        self.grip(None)
        time.sleep(5)
        self.attach('held_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
