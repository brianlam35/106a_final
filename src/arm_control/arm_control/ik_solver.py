import numpy as np
import ikpy.chain
import os
from ament_index_python.packages import get_package_share_directory

class IKSolver:
    def __init__(self):
        # 1. Dynamically find the path to the installed URDF
        # This looks in install/arm_control/share/arm_control/urdf/d1.urdf
        pkg_path = get_package_share_directory('arm_control')
        urdf_path = os.path.join(pkg_path, 'urdf', 'd1.urdf')

        # 2. Load the Chain
        # Note: We need to verify the link mask.
        # D1 usually has a base (inactive) + 6 motors (active) + gripper/tip (inactive)
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            active_links_mask=[False, True, True, True, True, True, True, False] 
        )

    def compute_ik(self, target_x, target_y, target_z):
        # ... (rest of your logic) ...
        # Remember to verify link count!
        target_position = [target_x, target_y, target_z]
        ik_solution = self.chain.inverse_kinematics(target_position)
        
        # Slice to get only the 6 joint angles
        return list(ik_solution[1:7])