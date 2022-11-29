import numpy as np

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class wx250s(ManipulatorModel):
    """
    Jaco is a kind and assistive robot created by Kinova

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/wx250s/robot.xml"), idn=idn)
        
        # Set joint damping
        self.set_joint_attribute(attrib="frictionloss", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01)))

    @property
    def default_mount(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        #return "JacoThreeFingerDexterousGripper"
        return "wx250sGripper"

    @property
    def default_controller_config(self):
        return "OSC_POSE"

    @property
    def init_qpos(self):
    	return np.array([2.692, -6.680, 4.000, 3.170, 1.550, 1.760, 3.142])
	
    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"
