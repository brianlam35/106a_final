import numpy as np
import ikpy.chain
import os
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R

class IKSolver:
    def __init__(self):
        # 1. Find URDF
        pkg_path = get_package_share_directory('arm_control')
        urdf_path = os.path.join(pkg_path, 'urdf', 'd1.urdf')

        # 2. Load Chain
        # Mask: [Base, J1, J2, J3, J4, J5, J6, Tip]
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            active_links_mask=[False, True, True, True, True, True, True, False] 
        )

    def compute_ik(self, target_x, target_y, target_z):
        target_position = [target_x, target_y, target_z]

        # --- 1. CALCULATE SMART YAW ---
        # Calculate angle to target so arm points naturally at the bottle
        yaw_to_target = np.arctan2(target_y, target_x)

        # --- 2. DEFINE ORIENTATION ---
        # y=Pitch (45 deg down), z=Yaw (Towards bottle)
        # Check your URDF: If 'y' is pitch, this works. 
        # If it curls sideways, change 'y' to 'x'.
        r = R.from_euler('zy', [yaw_to_target, 30], degrees=True)
        target_orientation = r.as_matrix()

        # --- 3. SOLVE ---
        ik_solution = self.chain.inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation,
            orientation_mode='all',
            max_iter=500
        )
        
        return list(ik_solution[1:7])