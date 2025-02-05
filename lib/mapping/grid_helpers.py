import cv2
import numpy as np
import pywavemap as wave
from pywavemap import InterpolationMode
from scipy.ndimage import binary_dilation

import lib.constants as CFG

def generate_node_indicies(map_min_cell_width: float, planar_spread: float, z_lower_lim: float, z_upper_lim: float) -> np.ndarray:
    """
    Generate node indices for a 3D grid based on the given parameters.

    Args:
        map_min_cell_width (float): Minimum cell width of the map.
        planar_spread (float): Spread in the x and y directions.
        z_lower_lim (float): Lower limit for the z-axis.
        z_upper_lim (float): Upper limit for the z-axis.

    Returns
    -------
        np.ndarray: Array of node indices.
    """
    # Generate range of z values to query
    z_range = np.arange(np.round(z_lower_lim / map_min_cell_width), np.round(z_upper_lim / map_min_cell_width) + 1)

    # Retrieve positions corresponding to a certain planar spread
    steps = np.round(planar_spread / map_min_cell_width)
    x = np.arange(-steps / 2, steps / 2)
    y = np.arange(-steps / 2, steps / 2)

    # Generate 3D meshgrid
    node_idx_mgrid = np.array(np.meshgrid(x, y, z_range))

    return node_idx_mgrid.T.reshape(-1, 3), x.shape[0], z_range.shape[0]

class DynamicOccupancyGrid:
    """Helper class for creating and updating an occupancy grid from a Wavemap environmental map."""

    def __init__(self):
        """Initialize the DynamicOccupancyGrid."""

        # Retrieve height bounds from RealSense config
        self.r_bc = np.array(CFG.REALSENSE_EXTRINSICS_R_BC).reshape(3, )

        # Retrieve lower and upper height limits
        self.z_lower_lim = CFG.MAPPING_GRID_Z_LOWER_LIM
        self.z_upper_lim = self.r_bc[2] + CFG.MAPPING_GRID_Z_UPPER_LIM_EPSILON

        # Retrieve remaining grid parameters for construction
        self.query_cell_width = CFG.MAPPING_GRID_GRID_CELL_SIZE
        self.planar_spread = CFG.MAPPING_GRID_DEFAULT_PLANAR_SPREAD

        # Retrieve physical body radius
        self.physical_body_rad = CFG.MAPPING_GRID_BODY_RADIUS + CFG.MAPPING_GRID_BODY_RADIUS_EPSILON
        self.physical_body_rad_cell_width = int(np.round(self.physical_body_rad / self.query_cell_width))

        # Construct node indices for lower body
        self.z_lower_body_lim = CFG.MAPPING_GRID_Z_LOWER_BODY_LIMIT
        self.lower_body_indicies, self.grid_length, self.lower_body_grid_height = generate_node_indicies(
            self.query_cell_width, self.planar_spread, self.z_lower_lim, self.z_lower_body_lim)

        # Construct node indices for upper body
        self.upper_body_indicies, self.grid_length, self.upper_body_grid_height = generate_node_indicies(
            self.query_cell_width, self.planar_spread, self.z_lower_body_lim, self.z_upper_lim)

        # Generate query height
        self.query_height = wave.convert.cell_width_to_height(self.query_cell_width, CFG.MAPPING_MAP_MIN_CELL_WIDTH)

        # Initialize query points for lower and upper body
        self.lower_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.lower_body_indicies), 1)), self.lower_body_indicies), axis=1)
        self.upper_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.upper_body_indicies), 1)), self.upper_body_indicies), axis=1)

        # Initialize occupancy grids as boolean arrays
        self.lower_body_occupancy_grid = np.zeros((self.grid_length, self.grid_length), dtype=bool)
        self.upper_body_occupancy_grid = np.zeros((self.grid_length, self.grid_length), dtype=bool)
        self.traversability_grid = np.zeros((self.grid_length, self.grid_length), dtype=bool)

        # Initialize occupied points
        self.occupied_points = []

        # Initialize video writer
        # self.video_writer = cv2.VideoWriter(
        #     'occupancy_grid.mp4',
        #     cv2.VideoWriter_fourcc(*'mp4v'),
        #     10,  # frames per second
        #     (self.grid_length, self.grid_length),  # frame size
        #     isColor=False  # grayscale
        # )

    def reset_occupancy_grid(self):
        """
        Reset occupancy grid to all zeros.
        """
        self.lower_body_occupancy_grid.fill(False)
        self.upper_body_occupancy_grid.fill(False)

    def reset_query_size(self, new_query_cell_width: float):
        """
        Reset query size to new query cell width.

        Args:
            new_query_cell_width (float): New cell width for the query.
        """
        # Update query cell width
        self.query_cell_width = new_query_cell_width

        # Update query height
        self.query_height = wave.convert.cell_width_to_height(self.query_cell_width, CFG.MAPPING_MAP_MIN_CELL_WIDTH)

        # Generate upper and lower body node indices
        self.lower_body_indicies, self.grid_length, self.lower_body_grid_height = generate_node_indicies(
            self.query_cell_width, self.planar_spread, self.z_lower_lim, self.z_lower_body_lim)

        self.upper_body_indicies, self.grid_length, self.upper_body_grid_height = generate_node_indicies(
            self.query_cell_width, self.planar_spread, self.z_lower_body_lim, self.z_upper_lim)

        # Update query points
        self.lower_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.lower_body_indicies), 1)), self.lower_body_indicies), axis=1)
        self.upper_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.upper_body_indicies), 1)), self.upper_body_indicies), axis=1)

    def update_occupancy_grid(self, map: wave.Map):
        """
        Update occupancy grid with new wavemap.

        Args:
            map (wave.Map): Wavemap object to query.
        """
        # Query wavemap for upper and lower body
        upper_body_log_odds = map.get_cell_values(self.upper_body_query_points)
        lower_body_log_odds = map.get_cell_values(self.lower_body_query_points)

        # Convert to probabilities for thresholding
        upper_body_prob = np.exp(upper_body_log_odds) / (1 + np.exp(upper_body_log_odds))
        lower_body_prob = np.exp(lower_body_log_odds) / (1 + np.exp(lower_body_log_odds))

        # Reshape to occupancy values to collapse along z-axis
        upper_body_occupancy_values_raw = upper_body_prob.reshape((self.upper_body_grid_height, self.grid_length, self.grid_length)).T
        lower_body_occupancy_values_raw = lower_body_prob.reshape((self.lower_body_grid_height, self.grid_length, self.grid_length)).T

        # Update occupancy grids
        self.upper_body_occupancy_grid = np.any(upper_body_occupancy_values_raw > CFG.MAPPING_GRID_PROBABILITY_THRESH, axis=2)
        self.lower_body_occupancy_grid = np.any(lower_body_occupancy_values_raw > CFG.MAPPING_GRID_PROBABILITY_THRESH, axis=2)

        # Update traversability grid
        self.traversability_grid = generate_traversability_grid(self.upper_body_occupancy_grid, self.lower_body_occupancy_grid, self.physical_body_rad_cell_width)

        # Flip the grids to match the coordinate frame
        self.traversability_grid = self.traversability_grid[:, :]
        self.upper_body_occupancy_grid = self.upper_body_occupancy_grid[:, :]

        # Convert the grid to a format suitable for OpenCV
        grid_image = (self.traversability_grid * 255).astype(np.uint8)

        # Write the frame to the video
        # self.video_writer.write(grid_image)

    def update_occupied_points(self, map: wave.Map):
        """
        Update occupied points with new wavemap.

        Args:
            map (wave.Map): Wavemap object to query.
        """
        # Identify occupied points in 3D space
        self.occupied_points = []

        # Generate z range
        z_range = np.arange(0.1, 6.0, self.query_cell_width*2)

        # Retrieve positions corresponding to a certain planar spread
        steps = np.round(self.planar_spread)
        # Generate x and y values
        x = np.arange(-steps / 2, steps / 2, self.query_cell_width*3)
        y = np.arange(-steps / 2, steps / 2, self.query_cell_width*3)

        # Generate 3D meshgrid
        node_idx_mgrid = np.array(np.meshgrid(x, y, z_range))
        node_idx_mgrid = node_idx_mgrid.T.reshape(-1, 3)
        points_log_odds = map.interpolate(node_idx_mgrid, InterpolationMode.NEAREST)

        # Convert log odds to probability
        points_prob = np.exp(points_log_odds) / (1 + np.exp(points_log_odds))
        
        # Define a threshold for occupied points
        threshold = 0.75 # CFG.MAPPING_GRID_PROBABILITY_THRESH

        # Find indices where probability is greater than the threshold
        occupied_indices = np.argwhere(points_prob > threshold).flatten()
        
        # Add these points to the occupied points list
        occupied_points = node_idx_mgrid[occupied_indices]

        # Extend the occupied_points list with the new points
        self.occupied_points = occupied_points.tolist()

    def __del__(self):
        """
        Destructor to release the video writer when the class is destroyed.
        """
        pass
        # self.video_writer.release()

def generate_traversability_grid(upper_body_occupancy: np.array, lower_body_occupancy: np.array, physical_body_rad: float) -> np.array:
    """
    Generate a traversability grid using binary dilation for setting occupied regions.

    Args:
        upper_body_occupancy (np.array): Upper body occupancy grid.
        lower_body_occupancy (np.array): Lower body occupancy grid.
        physical_body_rad (float): Physical body radius in cell width.

    Returns
    -------
        np.array: Traversability grid.
    """
    # Create a structuring element for dilation
    struct_elem = np.ones((int(physical_body_rad * 2 + 1), int(physical_body_rad * 2 + 1)), dtype=bool)

    # Dilate the lower body occupancy grid
    dilated_lower_body = binary_dilation(lower_body_occupancy, structure=struct_elem)

    # Combine with upper body occupancy
    traversability_grid = np.logical_or(upper_body_occupancy, dilated_lower_body)

    return traversability_grid
