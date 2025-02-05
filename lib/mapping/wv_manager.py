import os
import time

import numpy as np
import pywavemap as wave
from pymlg.numpy import SE3, SO3

import lib.constants as CFG
from lib.mapping.grid_helpers import DynamicOccupancyGrid
from lib.sensors.realsense.realsense_manager import RealSenseManager


class DepthWavemapManager:
    def __init__(self):
        """Initialize the DepthWavemapManager."""
        print("Initializing DepthWavemapManager!")

        # Instantiate occupancy grid
        self.occupancy_grid = DynamicOccupancyGrid()

        # Start RealSense retrieval pipeline
        self.rs_manager = RealSenseManager()

        # Retrieve extrinsics
        r_bc = np.array(CFG.REALSENSE_EXTRINSICS_R_BC).reshape(1, 3)
        phi_bc = np.array(CFG.REALSENSE_EXTRINSICS_PHI_BC).reshape(1, 3)

        # Compute rotation matrix from rotation vector
        C_bc = SO3.Exp(phi_bc)

        # Form transformation matrix to apply to pose transformation
        self.T_bc = SE3.from_components(C_bc, r_bc)

        # Update rate
        self.update_rate = CFG.MAPPING_UPDATE_RATE

        # Track current time
        self.t_k = None

        # Create a map
        self.local_submap = wave.Map.create({
            "type": "hashed_wavelet_octree",
            "min_cell_width": {
                "meters": CFG.MAPPING_MAP_MIN_CELL_WIDTH
            },
            "remove_blocks_beyond_distance": {
                "meters": CFG.MAPPING_MAP_BLOCK_REMOVAL_DISTANCE
            }
        })

        # Create a measurement integration pipeline
        self.pipeline = wave.Pipeline(self.local_submap)
        # Add map operations
        self.pipeline.add_operation({
            "type": "threshold_map",
            "once_every": {
                "seconds": CFG.MAPPING_MAP_THRESHOLD_DELTA
            }
        })
        self.pipeline.add_operation({
            "type": "prune_map",
            "once_every": {
                "seconds": CFG.MAPPING_MAP_PRUNING_DELTA
            }
        })

        self.last_occupied_points_update_time = time.time()

    def start_pipeline(self):
        """
        Start the RealSense pipeline and add a measurement integrator to the pipeline.
        """
        # Start RealSense pipeline
        self.rs_manager.start_pipeline()

        # Retrieve intrinsics
        self.realsense_intrinsics = self.rs_manager.get_intrinsics()

        # Add a measurement integrator
        self.pipeline.add_integrator(
            "my_integrator", {
                "projection_model": {
                    "type": "pinhole_camera_projector",
                    "width": self.realsense_intrinsics.width,
                    "height": self.realsense_intrinsics.height,
                    "fx": self.realsense_intrinsics.fx,
                    "fy": self.realsense_intrinsics.fy,
                    "cx": self.realsense_intrinsics.ppx,
                    "cy": self.realsense_intrinsics.ppy
                },
                "measurement_model": {
                    "type": "continuous_ray",
                    "range_sigma": {
                        "meters": CFG.MAPPING_MAP_RANGE_SIGMA
                    },
                    "scaling_free": CFG.MAPPING_MAP_SCALING_FREE,
                    "scaling_occupied": CFG.MAPPING_MAP_SCALING_OCCUPIED
                },
                "integration_method": {
                    "type": "hashed_wavelet_integrator",
                    "min_range": {
                        "meters": CFG.MAPPING_MAP_MIN_RANGE
                    },
                    "max_range": {
                        "meters": CFG.MAPPING_MAP_MAX_RANGE
                    },
                    "max_update_resolution": {
                        "meters": CFG.MAPPING_MAP_MAX_UPDATE_RESOLUTION
                    },
                },
            })

    def integrate_depth_image(self, t_k: float, T_ab_k: np.ndarray):
        """
        Integrate the current depth image into the local submap using a pose estimate.

        Parameters
        ----------
        t_k : float
            Current time step.
        T_ab_k : np.ndarray
            SE(3) pose estimate of the body in the inertial frame at timestep k.
        """
        # First, check if we need to update the map
        # if self.t_k is not None:
        #     if t_k - self.t_k < 1 / self.update_rate:
        #         return

        # Update to current time and update map
        self.t_k = t_k

        # Retrieve depth image
        depth_np = self.rs_manager.get_metric_depth_frame()

        # Convert depth image to wave image format
        image = wave.Image(depth_np.T)

        # Given T_ab, compute T_ac from extrinsics, and integrate image
        T_ac_k = T_ab_k @ self.T_bc
        T_ac_k = wave.Pose(T_ac_k)

        # Run the integration pipeline
        self.pipeline.run_pipeline(["my_integrator"], wave.PosedImage(T_ac_k, image))

        # After integrating, update occupancy grid
        self.occupancy_grid.update_occupancy_grid(self.local_submap)

        # Update occupied points
        if time.time() - self.last_occupied_points_update_time > 5.0:
            self.occupancy_grid.update_occupied_points(self.local_submap)
            self.last_occupied_points_update_time = time.time()

    def get_upper_body_occupancy_grid(self):
        """
        Retrieve the upper body occupancy grid.

        Returns
        -------
        np.ndarray
            Upper body occupancy grid.
        """
        return self.occupancy_grid.upper_body_occupancy_grid

    def get_lower_body_occupancy_grid(self):
        """
        Retrieve the lower body occupancy grid.

        Returns
        -------
        np.ndarray
            Lower body occupancy grid.
        """
        return self.occupancy_grid.lower_body_occupancy_grid

    def get_traversability_grid(self):
        """
        Retrieve the traversability grid.

        Returns
        -------
        np.ndarray
            Traversability grid.
        """
        return self.occupancy_grid.traversability_grid

    def get_flattened_upper_body_occupancy_grid(self):
        """
        Retrieve the flattened upper body occupancy grid.

        Returns
        -------
        np.ndarray
            Flattened upper body occupancy grid.
        """
        return self.occupancy_grid.upper_body_occupancy_grid.flatten()

    def get_flattened_lower_body_occupancy_grid(self):
        """
        Retrieve the flattened lower body occupancy grid.

        Returns
        -------
        np.ndarray
            Flattened lower body occupancy grid.
        """
        return self.occupancy_grid.lower_body_occupancy_grid.flatten()

    def get_flattened_traversability_grid(self):
        """
        Retrieve the flattened traversability grid.

        Returns
        -------
        np.ndarray
            Flattened traversability grid.
        """
        return self.occupancy_grid.traversability_grid.flatten()

    def get_occupied_points(self):
        """
        Retrieve the occupied points.

        Returns
        -------
        np.ndarray
            Occupied points.
        """
        return self.occupancy_grid.occupied_points

    def threshold_map(self):
        """
        Threshold the map to restrict log-odds to a specific range, if specified.
        """
        self.local_submap.threshold()

    def save_map(self):
        """
        Prune and save the current map to a file.
        """
        # Prune the map
        self.local_submap.prune()

        # Get current map name, in time
        map_name = "map_" + str(int(time.time())) + ".wvmp"

        # Save the map
        output_map_path = CFG.MAPPING_MAP_OUTPUT_MAP_PATH + map_name
        # Add /home/$USER to the beginning of the path
        output_map_path = f"/home/{os.getenv('USER')}/{output_map_path}"

        self.local_submap.store(output_map_path)

        # Clean up pipeline and map
        del self.pipeline, self.local_submap
