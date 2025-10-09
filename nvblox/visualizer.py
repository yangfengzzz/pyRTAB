#  Copyright (c) 2025 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

from typing import List, Dict, Deque
from collections import deque

import numpy as np
import numpy.typing as npt
import rerun as rr
import rerun.blueprint as rrb

from nvblox_torch.mesh import Mesh


# NOTE(alexmillane, 2025.05.22): This is a modified version of the RerunVisualizer class
# from the cuvslam examples. Right now the visualizer does not ship with the PyCuVSLAM
# package, so we need to copy it here. If they move it into their package we can import
# it from there and subclass it.

# pylint: disable=invalid-name


class RerunVisualizer:
    """Visualizer for slam and nvblox for the nvblox_torch realsense example."""

    def __init__(self) -> None:
        """Initialize rerun visualizer."""
        self._start_rerun_visualizer()
        # Parameters
        self.camera_pose_axis_scale = 0.1
        self.trajectory_length = 100
        # State
        self.track_colors: Dict[int, npt.NDArray] = {}
        self.t_W_C_history: Deque[npt.NDArray] = deque(maxlen=self.trajectory_length)

    def _start_rerun_visualizer(self) -> None:
        rr.init('SLAM Visualizer', spawn=True)
        rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
        rr.send_blueprint(rrb.Blueprint(
            rrb.TimePanel(state='collapsed'),
            rrb.Horizontal(column_shares=[0.5, 0.5],
                           contents=[
                               rrb.Spatial2DView(origin='world/camera_0'),
                               rrb.Spatial3DView(origin='world'),
                           ])),
            make_active=True)

    def _log_rig_pose(self, q_W_C: npt.NDArray, t_W_C: npt.NDArray) -> None:
        """Log rig pose to Rerun."""
        rr.log(
            'world/camera_0',
            rr.Transform3D(translation=t_W_C, quaternion=q_W_C),
            rr.Arrows3D(
                vectors=np.eye(3) * self.camera_pose_axis_scale,
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]  # RGB for XYZ axes
            ))

    def _log_trajectory(self) -> None:
        """Log the trajectory to Rerun."""
        rr.log('world/trajectory', rr.LineStrips3D(self.t_W_C_history), static=True)

    def _visualize_nvblox_mesh(self, mesh: Mesh) -> None:
        rr.log(
            'world/mesh',
            rr.Mesh3D(
                vertex_positions=mesh.vertices().cpu().numpy(),
                vertex_colors=mesh.vertex_colors().cpu().numpy(),
                triangle_indices=mesh.triangles().cpu().numpy(),
            ))

    def visualize_nvblox(self, mesh: Mesh) -> None:
        """Visualize the nvblox mesh.

        Args:
            mesh: The nvblox mesh to visualize.
        """
        self._visualize_nvblox_mesh(mesh)
