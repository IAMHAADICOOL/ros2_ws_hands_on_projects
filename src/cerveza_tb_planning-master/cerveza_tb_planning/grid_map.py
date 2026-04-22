import numpy as np
import importlib
try:
    bresenham = importlib.import_module('bresenham').bresenham
except ImportError:
    def bresenham(x0, y0, x1, y1):
        """Yield integer grid cells along a line using Bresenham's algorithm."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            yield (x, y)
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

from nav_msgs.msg import OccupancyGrid      # For receiving robot position and publish map
from scipy.ndimage import binary_dilation   # For map inflation

class GridMap:
    # Saturation log-odds limits (to avoid inf values)
    LMAX = 6.91     # Equivelent to 0.999 probability
    LMIN = -6.91    # Equivalent to 0.001 probability

    def __init__(self, center, cell_size=0.01, map_size=20):
        self.cell_size = cell_size  # Map resolution [m]
        self.grid = np.zeros((int(map_size/cell_size), int(map_size/cell_size))) # 0 -> 0.5 log-odds
        self.origin = np.array(center) - np.array([map_size, map_size])/2        # Map bottom-left corner

    def get_map(self):
        return self.grid

    def get_origin(self):
        return self.origin
    
    def get_min_x(self):
        return self.origin[0]

    def get_min_y(self):
        return self.origin[1]

    def get_max_x(self):
        return self.origin[0] + self.grid.shape[0] * self.cell_size
    
    def get_max_y(self):
        return self.origin[1] + self.grid.shape[1] * self.cell_size
    
    # Converts a world position [m] into a grid index (cell)
    def position_to_cell(self, position):
        return ((np.array(position) - self.origin) / self.cell_size).astype(int)
    
    def get_cell_value(self, cell):
        u, v = cell
        if 0 <= u < self.grid.shape[0] and 0 <= v < self.grid.shape[1]:
            return self.grid[u][v]
        else:
            return None  # Out of bounds

    def __update_cell__(self, uv, p):
        if p <= 0 or p >= 1:    # Check for valid probabilities
            return
        l = np.log(p / (1 - p)) # Inverse sensor model (prob -> log-odds)
        u, v = uv
        if 0 <= u < self.grid.shape[0] and 0 <= v < self.grid.shape[1]:                 # Bounds check
            self.grid[u][v] = np.clip(self.grid[u][v] + l, GridMap.LMIN, GridMap.LMAX)  # Update and clip cell

    def add_ray(self, ray_init_position, ray_angle, ray_range, p):
        # Calculate ray end position
        ray_end_position = [ray_init_position[0] + ray_range * np.cos(ray_angle),
                            ray_init_position[1] + ray_range * np.sin(ray_angle)]
        
        # Converts ray (start, end) points into grid cells (start, end)
        cell_start = self.position_to_cell(ray_init_position)
        cell_end = self.position_to_cell(ray_end_position)

        # Get all the intermediate cells from start cell to end cell (straight line) and set them as free
        cells = list(bresenham(cell_start[0], cell_start[1], cell_end[0], cell_end[1]))
        for cell in cells[:-1]:
            self.__update_cell__(cell, 1 - p)   # Intermediate cells (l - p -> 0.1) free
        self.__update_cell__(cells[-1], p)      # Last cell (0.9) obstacle

    def inflateObstacles(self, inflation_radius):
        inflation_cells = int(inflation_radius / self.cell_size)
        
        # Create circular structuring element
        y, x = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
        structure = (x**2 + y**2) <= inflation_cells**2
        
        # Dilate obstacles
        obstacle_mask = self.grid > 0.5
        inflated_mask = binary_dilation(obstacle_mask, structure=structure)
        
        inflated_grid = np.copy(self.grid)
        inflated_grid[inflated_mask & (self.grid <= 0.5)] = 1.0
        return inflated_grid
        

def map_to_msg(grid_map, inflated=False, inflation_radius=0.28):
    # Convert log-odds to occupancy values [0-100], -1 for unknown
    if inflated:
        grid = grid_map.inflateObstacles(inflation_radius)
    else:
        grid = grid_map.get_map()
    occupancy = np.full(grid.shape, -1, dtype=np.int8) # unknown (gray)
    occupancy[grid > 0.5] = 100   # occupied (black)
    occupancy[grid < -0.5] = 0    # free (white)

    msg = OccupancyGrid()
    msg.info.resolution = grid_map.cell_size        # Meters per cell
    msg.info.width = grid_map.grid.shape[0]         # Grid dimensions ...
    msg.info.height = grid_map.grid.shape[1]        
    msg.info.origin.position.x = grid_map.origin[0]  # Bottom-left corner position ...
    msg.info.origin.position.y = grid_map.origin[1]
    msg.data = occupancy.T.flatten().tolist()   # Convert to list for msg

    return msg

def msg_to_map(msg):
    origin = (msg.info.origin.position.x, msg.info.origin.position.y)
    center = (origin[0] + msg.info.width * msg.info.resolution / 2, 
              origin[1] + msg.info.height * msg.info.resolution / 2)
    grid_map = GridMap(center=center, cell_size=msg.info.resolution, 
                       map_size=max(msg.info.width, msg.info.height) * msg.info.resolution)
    
    # Convert using numpy
    data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width).T
    grid_map.grid[data == 0] = -1      # Free
    grid_map.grid[data == 100] = 100   # Occupied
    grid_map.grid[data == -1] = 0      # Unknown
    
    return grid_map