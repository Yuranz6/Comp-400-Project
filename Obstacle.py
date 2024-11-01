import numpy as np
import pygame

class Obstacles:
    def __init__(self, num_obstacles, map_size, velocity_range=(-0.04, 0.04)):
        """
        Initialize moving obstacles with smooth grid-based movement
        
        Args:
            num_obstacles: number of obstacles
            map_size: size of the grid (rows/cols)
            velocity_range: range of random velocities (in grid cells per frame)
        """
        self.num_obstacles = num_obstacles
        self.map_size = map_size
        self.velocity_range = velocity_range
        
        # Positions (as floating point for smooth movement)
        self.positions_x = []  # Current x positions
        self.positions_y = []  # Current y positions
        self.velocities_x = []  # Velocity in x direction
        self.velocities_y = []  # Velocity in y direction
        
        # Grid positions (integer positions for collision detection)
        self.grid_positions_x = []
        self.grid_positions_y = []
        
        # Initialize random positions and velocities
        self.initialize_obstacles()
        
        # Boundary handling
        self.buffer_zone = 1  # Cells from edge where obstacles will bounce
        
    def initialize_obstacles(self):
        """Initialize random positions and velocities for obstacles"""
        for i in range(self.num_obstacles):
            # Random position (avoiding edges)
            x = np.random.randint(10, self.map_size - 10)
            y = np.random.randint(10, self.map_size - 10)
            
            # Random velocity (alternating directions)
            vx = np.random.uniform(*self.velocity_range) * (-1)**i
            vy = np.random.uniform(*self.velocity_range) * (-1)**i
            
            self.positions_x.append(float(x))
            self.positions_y.append(float(y))
            self.grid_positions_x.append(int(x))
            self.grid_positions_y.append(int(y))
            self.velocities_x.append(vx)
            self.velocities_y.append(vy)
    
    def update_positions(self, true_map):
        """
    Update obstacle positions based on velocities and handle collisions
    
    Args:
        true_map: Map object containing static barriers
    """
        for i in range(self.num_obstacles):
            # Store old position
            old_x = self.positions_x[i]
            old_y = self.positions_y[i]
            
            # Update positions
            new_x = old_x + self.velocities_x[i]
            new_y = old_y + self.velocities_y[i]
            
            # Check collisions with walls
            if (new_x <= self.buffer_zone or 
                new_x >= self.map_size - self.buffer_zone):
                self.velocities_x[i] *= -1  # Reverse x velocity
                new_x = np.clip(new_x, self.buffer_zone, 
                            self.map_size - self.buffer_zone)
                
            if (new_y <= self.buffer_zone or 
                new_y >= self.map_size - self.buffer_zone):
                self.velocities_y[i] *= -1  # Reverse y velocity
                new_y = np.clip(new_y, self.buffer_zone, 
                            self.map_size - self.buffer_zone)
            
            
            # Check collisions with static barriers
            grid_x = int(new_x)
            grid_y = int(new_y)
            
            # TODO: improve bouncing logic using reflection vector
            # TODO: this bouncing logic is buggy, need further investigation
            # Check if new position would hit a barrier
            if true_map.grid[grid_x][grid_y].is_barrier():
                # If moving more in x direction, bounce in x
                if abs(self.velocities_x[i]) > abs(self.velocities_y[i]):
                    self.velocities_x[i] *= -1
                    new_x = old_x  # Stay at old x position
                # If moving more in y direction, bounce in y
                else:
                    self.velocities_y[i] *= -1
                    new_y = old_y  # Stay at old y position
            
            # Update positions
            self.positions_x[i] = new_x
            self.positions_y[i] = new_y
            self.grid_positions_x[i] = int(new_x)
            self.grid_positions_y[i] = int(new_y)
    
    def get_obstacle_positions(self):
        """Return current grid positions of all obstacles"""
        return (self.grid_positions_x, self.grid_positions_y)
    
    def get_exact_positions(self):
        """Return exact floating point positions"""
        return (self.positions_x, self.positions_y)
    
    def get_obstacle_velocities(self):
        """Return current velocities of all obstacles"""
        return (self.velocities_x, self.velocities_y)
    
    def draw(self, win, cell_size):
        """
        Draw obstacles on pygame window with smooth movement
        
        Args:
            win: pygame window surface
            cell_size: size of each grid cell
        """
        for i in range(self.num_obstacles):
            # Use exact positions for drawing
            x = self.positions_x[i] * cell_size + cell_size/2
            y = self.positions_y[i] * cell_size + cell_size/2
            
            # Draw obstacle
            pygame.draw.circle(win, (0, 0, 0), (int(x), int(y)), cell_size/3)
            
            # Draw velocity vector
            end_x = x + self.velocities_x[i] * cell_size * 10
            end_y = y + self.velocities_y[i] * cell_size * 10
            pygame.draw.line(win, (255, 0, 0), (int(x), int(y)), 
                           (int(end_x), int(end_y)), 2)