#  Copyright (c) 2025 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

import time

import torch
import argparse
import cuvslam as vslam
import sys

from nvblox_torch.examples.realsense.realsense_utils import rs_intrinsics_to_matrix, rs_extrinsics_to_homogeneous
from nvblox_torch.examples.realsense.realsense_dataloader import RealsenseDataloader
from nvblox_torch.examples.realsense.vslam_utils import get_vslam_stereo_rig, to_homogeneous
from nvblox_torch.examples.realsense.visualizer import RerunVisualizer
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
from nvblox_torch.mapper import Mapper
from nvblox_torch.mapper_params import MapperParams, ProjectiveIntegratorParams
from nvblox_torch.timer import Timer, timer_status_string

# pylint: disable=invalid-name

PRINT_TIMING_EVERY_N_SECONDS = 1.0


def parse_args() -> argparse.Namespace:
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--max_frames',
                        type=int,
                        default=5000,
                        help='Maximum number of frames to process.')
    parser.add_argument('--voxel_size_m',
                        type=float,
                        default=0.01,
                        help='Voxel size for the nvblox mapper (in meters).')
    parser.add_argument('--max_integration_distance_m',
                        type=float,
                        default=1.0,
                        help='Maximum integration distance for the nvblox mapper (in meters).')
    parser.add_argument('--visualize_mesh_hz',
                        type=float,
                        default=5,
                        help='Update and visualize mesh at this frequency (Hz).')
    return parser.parse_args()


def main() -> int:
    """
    This example demonstrates the integration of NVIDIA's nvblox mapping system with
    Intel RealSense cameras and cuvslam for visual odometry tracking.
    It captures depth and color data from a RealSense camera, tracks its pose using cuvslam
    and builds a 3D reconstruction using nvblox.
    """
    args = parse_args()

    realsense_dataloader = RealsenseDataloader(max_steps=args.max_frames)

    # Set up cuvslam tracker
    cfg = vslam.TrackerConfig(async_sba=False,
                              enable_final_landmarks_export=True,
                              odometry_mode=vslam.TrackerOdometryMode.Multicamera,
                              horizontal_stereo_camera=False)
    rig = get_vslam_stereo_rig(realsense_dataloader.left_infrared_intrinsics(),
                               realsense_dataloader.right_infrared_intrinsics(),
                               realsense_dataloader.T_C_left_infrared_C_right_infrared())
    cuvslam_tracker = vslam.Tracker(rig, cfg)

    # Create some parameters
    projective_integrator_params = ProjectiveIntegratorParams()
    projective_integrator_params.projective_integrator_max_integration_distance_m = \
        args.max_integration_distance_m
    mapper_params = MapperParams()
    mapper_params.set_projective_integrator_params(projective_integrator_params)

    # Initialize nvblox mapper
    nvblox_mapper = Mapper(voxel_sizes_m=args.voxel_size_m,
                           integrator_types=ProjectiveIntegratorType.TSDF,
                           mapper_parameters=mapper_params)

    # Set up some constants such as camera extrinsics and intrinsics
    T_C_left_infrared_C_color = realsense_dataloader.T_C_left_infrared_C_color()
    T_C_left_infrared_C_color = rs_extrinsics_to_homogeneous(T_C_left_infrared_C_color)
    T_C_left_infrared_C_color = torch.from_numpy(T_C_left_infrared_C_color).float()
    depth_intrinsics = torch.from_numpy(
        rs_intrinsics_to_matrix(realsense_dataloader.depth_intrinsics())).float()
    color_intrinsics = torch.from_numpy(
        rs_intrinsics_to_matrix(realsense_dataloader.color_intrinsics())).float()

    # Visualize in rerun
    visualizer = RerunVisualizer()

    last_print_time = time.time()
    last_visualize_mesh_time = time.time()
    dataload_timer = None
    T_W_C_left_infrared = None
    for frame in realsense_dataloader:
        if dataload_timer is not None:
            dataload_timer.stop()

        # Track camera pose using cuvslam on the left infrared camera
        with Timer('cuvslam'):
            if frame['left_infrared_image'] is not None and frame[
                'right_infrared_image'] is not None:
                T_W_C_left_infrared = cuvslam_tracker.track(
                    frame['timestamp'],
                    (frame['left_infrared_image'], frame['right_infrared_image']))
                T_W_C_left_infrared = to_homogeneous(T_W_C_left_infrared.pose.translation,
                                                     T_W_C_left_infrared.pose.rotation)
                T_W_C_left_infrared = torch.from_numpy(T_W_C_left_infrared).float()

        # Do reconstruction using the depth
        with Timer('depth'):
            if frame['depth'] is not None and \
                    T_W_C_left_infrared is not None:
                # TODO(alexmillane, 2025.05.22): The pose used here is slighly wrong. It should be
                # interpolated between the cuVSLAM poses based on the timestamp.
                nvblox_mapper.add_depth_frame(frame['depth'], T_W_C_left_infrared, depth_intrinsics)

        with Timer('color'):
            if T_W_C_left_infrared is not None and \
                    frame['rgb'] is not None:
                # Convert the left infrared camera pose to the color camera frame
                T_W_C_color = T_W_C_left_infrared @ T_C_left_infrared_C_color
                # TODO(alexmillane, 2025.05.22): The pose used here is slighly wrong. It should be
                # interpolated between the cuVSLAM poses based on the timestamp.
                nvblox_mapper.add_color_frame(frame['rgb'], T_W_C_color, color_intrinsics)

        with Timer('visualize_rerun'):
            # Visualize pose. This occurs every time we track.
            if T_W_C_left_infrared is not None and frame['left_infrared_image'] is not None:
                visualizer.visualize_cuvslam(T_W_C_left_infrared.cpu().numpy(),
                                             frame['left_infrared_image'],
                                             cuvslam_tracker.get_last_observations(0))
            # Visualize mesh. This is performed at an (optionally) reduced rate.
            current_time = time.time()
            if (current_time - last_visualize_mesh_time) >= (1.0 / args.visualize_mesh_hz):
                with Timer('mesh/update'):
                    nvblox_mapper.update_color_mesh()
                with Timer('mesh/to_cpu'):
                    color_mesh = nvblox_mapper.get_color_mesh()
                with Timer('visualize/mesh'):
                    visualizer.visualize_nvblox(color_mesh)
                last_visualize_mesh_time = current_time

        # Print timing statistics
        current_time = time.time()
        if current_time - last_print_time >= PRINT_TIMING_EVERY_N_SECONDS:
            print(timer_status_string())
            last_print_time = current_time

        # This timer times how long it takes to get the next frame
        dataload_timer = Timer('dataload')

    # Print final timing statistics
    print(timer_status_string())

    print('Done')

    return 0


if __name__ == '__main__':
    sys.exit(main())
