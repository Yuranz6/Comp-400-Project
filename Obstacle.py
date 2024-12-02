import numpy as np
import pygame
class Obstacles:
    def __init__(self, num_obstacles=30, map_size=50):
        self.num_obstacles = num_obstacles
        self.map_size = map_size
        self.positions = []  # Current positions
        self.velocities = []  # Movement velocities
        self.initialize_obstacles()

    def initialize_obstacles(self):
        """Initialize obstacles with random positions and velocities"""
        self.positions = []
        self.velocities = []
        
        for _ in range(self.num_obstacles):
            while True:
                # Random position anywhere on the map
                pos = [np.random.randint(0, self.map_size), 
                    np.random.randint(0, self.map_size)]
                
                # Check if position is not too close to other obstacles
                valid_pos = True
                for existing_pos in self.positions:
                    if (abs(pos[0] - existing_pos[0]) < 2 and 
                        abs(pos[1] - existing_pos[1]) < 2):
                        valid_pos = False
                        break
                
                if valid_pos:
                    break
            
            # Random velocity (-1, 0, or 1 for both x and y)
            vel = [np.random.randint(-1, 2), 
                np.random.randint(-1, 2)]
            
            # Ensure velocity is not zero
            if vel[0] == 0 and vel[1] == 0:
                vel[np.random.randint(0, 2)] = np.random.choice([-1, 1])
            
            self.positions.append(pos)
            self.velocities.append(vel)

    def update_positions(self, map_obj):
        """Update obstacle positions with realistic wall bouncing"""
        for i in range(self.num_obstacles):
            current_pos = self.positions[i]
            velocity = self.velocities[i]
            
            # Calculate new position
            new_x = current_pos[0] + velocity[0]
            new_y = current_pos[1] + velocity[1]
            
            # Handle edge bouncing
            if new_x < 0:
                new_x = 0
                self.velocities[i][0] = abs(velocity[0])  # Bounce right
            elif new_x >= self.map_size:
                new_x = self.map_size - 1
                self.velocities[i][0] = -abs(velocity[0])  # Bounce left
                
            if new_y < 0:
                new_y = 0
                self.velocities[i][1] = abs(velocity[1])  # Bounce down
            elif new_y >= self.map_size:
                new_y = self.map_size - 1
                self.velocities[i][1] = -abs(velocity[1])  # Bounce up
            
            # Handle wall collisions
            if map_obj is not None:
                # Check horizontal wall collision
                if (map_obj.grid[new_x][current_pos[1]].is_barrier()):
                    self.velocities[i][0] = -velocity[0]  # Reverse x velocity only
                    new_x = current_pos[0]  # Stay at current x
                
                # Check vertical wall collision
                if (map_obj.grid[current_pos[0]][new_y].is_barrier()):
                    self.velocities[i][1] = -velocity[1]  # Reverse y velocity only
                    new_y = current_pos[1]  # Stay at current y
            
            # Check for other dynamic obstacles
            for j, other_pos in enumerate(self.positions):
                if i != j:
                    if (abs(new_x - other_pos[0]) < 2 and 
                        abs(new_y - other_pos[1]) < 2):
                        # Random new direction on obstacle collision
                        directions = [
                            (-1, -1), (-1, 0), (-1, 1),
                            (0, -1),           (0, 1),
                            (1, -1),  (1, 0),  (1, 1)
                        ]
                        new_dir = directions[np.random.randint(len(directions))]
                        self.velocities[i] = list(new_dir)
                        break
            
            # Update position
            self.positions[i] = [new_x, new_y]
        
    def get_obstacle_positions(self):
        """Return current obstacle positions as two lists of x and y coordinates"""
        x_coords = [pos[0] for pos in self.positions]
        y_coords = [pos[1] for pos in self.positions]
        return x_coords, y_coords
    
    def draw(self, surface, cell_size):
        """Draw obstacles on the given surface"""
        for pos in self.positions:
            x = pos[1] * cell_size + cell_size // 2
            y = pos[0] * cell_size + cell_size // 2
            pygame.draw.circle(surface, (255, 165, 0), (x, y), cell_size // 3)