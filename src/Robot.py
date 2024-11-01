import numpy as np
import pygame 
from src.Map import Map
from Obstacle import Obstacles


class Robot:
    def __init__(self, true_map, sensor_range=5):
        # Position and Movement
        self.current_pos = None
        self.orientation = 0  # in radians, 0 is facing right
        self.velocity = 0
        self.max_velo = 10 # moves 10 grids max
        self.min_velo = 1
        
        # Map References
        self.true_map = true_map  # Ground truth map
        self.perceived_map = Map()  # Robot's understanding of environment
        self.sensor_range = sensor_range
        
        # Navigation
        self.current_path = []
        self.path_index = 0
        self.goal = None
        
        # Obstacles
        self.obstacles = Obstacles(num_obstacles=10, map_size=self.true_map.rows)
        
        # Path Planning
        self.path = []
        self.waypoints = []
        self.current_waypoint_idx = 0
    
    def load_map(self):
        '''Load true map from storage into robot's memory and add noise to it to make the perceived map'''
        self.perceived_map.load_map() # TODO: add noise
        
        
        
    def update_perception(self):
        """Update perceived map and detect obstacles within sensor range"""
        row, col = self.current_pos
        
        # Clear previous dynamic obstacles from perceived map to draw the updated pos of obstacles
        for i in range(self.perceived_map.rows):
            for j in range(self.perceived_map.rows):
                if self.perceived_map.grid[i][j].is_barrier() and not self.true_map.grid[i][j].is_barrier():
                    self.perceived_map.grid[i][j].reset()
        
        # First update static obstacles from true map
        for i in range(-self.sensor_range, self.sensor_range + 1):
            for j in range(-self.sensor_range, self.sensor_range + 1):
                new_row, new_col = row + i, col + j
                
                if (0 <= new_row < self.true_map.rows and 
                    0 <= new_col < self.true_map.rows and
                    abs(i) + abs(j) <= self.sensor_range):
                    
                    true_spot = self.true_map.grid[new_row][new_col]
                    perceived_spot = self.perceived_map.grid[new_row][new_col]
                    
                    if true_spot.is_barrier():
                        perceived_spot.make_barrier()
                    elif true_spot.is_start():
                        perceived_spot.make_start()
                    elif true_spot.is_end():
                        perceived_spot.make_end()
        
         # Then detect dynamic obstacles within sensor range
        ox, oy = self.obstacles.get_obstacle_positions()  # Gets grid positions
        for x, y in zip(ox, oy):
            dx = x - row
            dy = y - col
            if abs(dx) + abs(dy) <= self.sensor_range:
                if (0 <= x < self.true_map.rows and 
                    0 <= y < self.true_map.rows):
                    self.perceived_map.grid[x][y].make_barrier()
    
    # ------------------------A* PATH PLANNING----------------------------------
    # Reference code: https://github.com/Shubhranshu153/Obstacle_Avoidance_MPC_Controller_A_star_Planning
    # with slight modifications
    def global_path_planner(self):
        """Generate initial optimal path using A* + discretize into waypoints
        """
        if not self.true_map.end:
            return False
            
        start = self.current_pos
        goal = self.perceived_map.end.get_pos()
        
        # Initialize A* lists
        open_list = []  # Nodes to be evaluated
        closed_list = []  # Nodes already evaluated
        came_from = {}  # Keep track of best previous node
        
        # Cost dictionaries
        g_score = {}  # Cost from start to node
        f_score = {}  # Estimated total cost (g_score + heuristic)
        
        # Initialize start node
        start_node = tuple(start)  # Convert to tuple for hashing
        open_list.append(start_node)
        g_score[start_node] = 0
        f_score[start_node] = self.heuristic(start, goal)
        
        while open_list:
            # Get node with lowest f_score
            current = min(open_list, key=lambda x: f_score.get(x, float('inf')))
            
            if current == tuple(goal):
                # Path found, reconstruct it
                self.path = self.reconstruct_path(came_from, current)
                self.discretize_path()
                return True
                
            open_list.remove(current)
            closed_list.append(current)
            
            # Check all neighbors
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (-1,-1), (1,-1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip if out of bounds or barrier
                if not (0 <= neighbor[0] < self.true_map.rows and 
                       0 <= neighbor[1] < self.true_map.rows):
                    continue
                    
                if self.true_map.grid[neighbor[0]][neighbor[1]].is_barrier():
                    continue
                    
                if neighbor in closed_list:
                    continue
                
                # Calculate g_score for this neighbor
                tentative_g = g_score[current]
                if dx*dy != 0:  # Diagonal movement
                    tentative_g += 1.414  # sqrt(2)
                else:  # Cardinal movement
                    tentative_g += 1.0
                
                if neighbor not in open_list:
                    open_list.append(neighbor)
                elif tentative_g >= g_score.get(neighbor, float('inf')):
                    continue
                
                # This path is the best so far
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
        
        return False  # No path found
        
    def heuristic(self, a, b):
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
            

    
    def discretize_path(self, spacing=5):
        """Convert A* path into equally spaced waypoints"""
        if not self.path:
            return
            
        # Always include start point
        self.waypoints = [self.path[0]]
        
        # Add intermediate points with specified spacing
        dist_accumulated = 0
        for i in range(1, len(self.path)):
            prev = self.path[i-1]
            curr = self.path[i]
            
            # Calculate distance between points
            segment_dist = ((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)**0.5
            dist_accumulated += segment_dist
            
            # Add point if we've exceeded spacing
            if dist_accumulated >= spacing:
                self.waypoints.append(curr)
                dist_accumulated = 0
        
        # Always include goal point
        if self.path[-1] not in self.waypoints:
            self.waypoints.append(self.path[-1])
    
    # ---------------------------END OF A* PATH PLANNING----------------------------
    
    # ---------------------------(windowed) Artificial Potential Field(AFP) for local replanning----------------
    
    def calculate_window_potential(self):
        """
        Calculate APF based on:
        1. Attractive force only from closest waypoint
        2. Repulsive forces from dynamic obstacles within sensor window
        """
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints) - 1:
            return None
            
        # Parameters
        K_att = 1.0  # Attractive potential gain
        K_rep = 100.0  # Repulsive potential gain
        d0 = 5.0  # Influence range of obstacles
        
        curr_x, curr_y = self.current_pos
        
        # Get closest waypoint as intermediate goal
        next_waypoint = self.waypoints[self.current_waypoint_idx + 1]
        
        # Calculate attractive potential to current intermediate goal
        goal_x, goal_y = next_waypoint
        dist_to_goal = np.hypot(goal_x - curr_x, goal_y - curr_y)
        
        # If very close to current waypoint, shift to next one
        if dist_to_goal < 1.0:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints) - 1:
                return None
            next_waypoint = self.waypoints[self.current_waypoint_idx + 1]
            goal_x, goal_y = next_waypoint
            dist_to_goal = np.hypot(goal_x - curr_x, goal_y - curr_y)
        
        # Attractive force towards current intermediate goal
        f_att_x = K_att * (goal_x - curr_x) / dist_to_goal if dist_to_goal > 0 else 0
        f_att_y = K_att * (goal_y - curr_y) / dist_to_goal if dist_to_goal > 0 else 0
        
        # Only consider dynamic obstacles within sensor window for repulsive forces
        f_rep_x = 0
        f_rep_y = 0
        
        # Get positions of dynamic obstacles
        ox, oy = self.obstacles.get_obstacle_positions()
        
        # Check each obstacle within sensor window
        for obs_x, obs_y in zip(ox, oy):
            dist_to_obs = np.hypot(obs_x - curr_x, obs_y - curr_y)
            
            # Only consider obstacles within sensor range
            if dist_to_obs <= self.sensor_range:
                if dist_to_obs <= d0:  # Within influence range
                    f_rep_mag = K_rep * (1/dist_to_obs - 1/d0) * (1/dist_to_obs**2) # TODO: maybe use exponential is better? 
                    
                    # Direction from obstacle to robot
                    f_rep_x += f_rep_mag * (curr_x - obs_x) / dist_to_obs
                    f_rep_y += f_rep_mag * (curr_y - obs_y) / dist_to_obs
        
        # Combine forces
        total_fx = f_att_x + f_rep_x
        total_fy = f_att_y + f_rep_y
        
        # Normalize force vector
        force_magnitude = np.hypot(total_fx, total_fy)
        if force_magnitude > 0:
            total_fx = total_fx / force_magnitude
            total_fy = total_fy / force_magnitude
        
        return total_fx, total_fy, force_magnitude
    
    # ---------------------------END OF LOCAL REPLANNING----------------
    # ---------------------------ROBOT NAVIGATION----------------------------------
    def move(self):
        """Move robot along the planned path"""
        if not self.path or not self.waypoints:
            print("No path to follow!")
            return False
            
        # Get current and next waypoint
        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            print("Reached goal!")
            return False
            
        current = self.current_pos
        next_point = self.waypoints[self.current_waypoint_idx + 1]
        
        # Check if we've reached current waypoint
        dist_to_waypoint = ((current[0] - next_point[0])**2 + 
                        (current[1] - next_point[1])**2)**0.5
        
        if dist_to_waypoint < 1.0:  # Within 1 grid cell of waypoint
            self.current_waypoint_idx += 1
            return True
        
        # Calculate movement direction
        dx = next_point[0] - current[0]
        dy = next_point[1] - current[1]
        
        # Normalize to get unit direction
        length = (dx**2 + dy**2)**0.5
        if length > 0:
            dx = dx/length
            dy = dy/length
        
        # Move one grid cell in the direction of next waypoint
        new_x = int(round(current[0] + dx))
        new_y = int(round(current[1] + dy))
        
        # Check if new position is valid
        if (0 <= new_x < self.true_map.rows and 
            0 <= new_y < self.true_map.rows and 
            not self.true_map.grid[new_x][new_y].is_barrier()):
            self.current_pos = (new_x, new_y)
            return True
            
        return False
    
    
    def draw(self, win):
        """Visualize robot, sensor range, and perceived map"""
        # Draw perceived map
        self.perceived_map.draw(win)
        
        # Draw A* path
        if self.path:
            for i in range(len(self.path) - 1):
                start = self.path[i]
                end = self.path[i + 1]
                start_x = start[1] * self.true_map.gap + self.true_map.gap // 2
                start_y = start[0] * self.true_map.gap + self.true_map.gap // 2
                end_x = end[1] * self.true_map.gap + self.true_map.gap // 2
                end_y = end[0] * self.true_map.gap + self.true_map.gap // 2
                pygame.draw.line(win, (0, 255, 0), (start_x, start_y), 
                            (end_x, end_y), 2)
        
        # Draw waypoints
        for i, point in enumerate(self.waypoints):
            x = point[1] * self.true_map.gap + self.true_map.gap // 2
            y = point[0] * self.true_map.gap + self.true_map.gap // 2
            color = (0, 0, 255) if i != self.current_waypoint_idx else (255, 165, 0)
            pygame.draw.circle(win, color, (x, y), 5)
        
        # Draw robot
        row, col = self.current_pos
        center_x = col * self.true_map.gap + self.true_map.gap // 2
        center_y = row * self.true_map.gap + self.true_map.gap // 2
        pygame.draw.circle(win, (255, 0, 0), (center_x, center_y), 
                        self.true_map.gap // 3)
        
        # Draw sensor range
        sensor_radius = self.sensor_range * self.true_map.gap
        pygame.draw.circle(win, (200, 200, 200), (center_x, center_y), 
                        sensor_radius, 1)
                
                
'''

TESTING ----------------------------------------------------------------


'''
def main():
    width = 800
    map = Map(width)
    map_drawn = False
    robot = Robot(true_map=map, sensor_range=5)
    win = pygame.display.set_mode((width, width))
    clock = pygame.time.Clock()
    
    running = True
    while running:
        clock.tick(60)  # 60 FPS
        
        # If robot hasn't loaded a map yet, draw UI for map creation
        if not map_drawn:
            map.draw(win)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                # Handle map creation events
                if pygame.mouse.get_pressed()[0]:
                    pos = pygame.mouse.get_pos()
                    map.handle_click(pos, 1)
                    
                elif pygame.mouse.get_pressed()[2]:
                    pos = pygame.mouse.get_pos()
                    map.handle_click(pos, 3)

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_s and map.start and map.end:
                        map.save_map()
                        map_drawn = True
                        robot.load_map()
                        robot.current_pos = map.start.get_pos()
                        robot.update_perception()
                        if robot.global_path_planner():
                            print('path found! Intermediate waypoints: ', robot.waypoints)
                        else:
                            print('No path found!')
                    if event.key == pygame.K_c:
                        map.clear()
                        map_drawn = False
                        
        # If robot has loaded a map, run navigation
        else:
            # Clear screen or redraw background
            win.fill((255, 255, 255))  
            
            
            # Update and draw everything in order:
            
            # 1. Update obstacle positions
            robot.obstacles.update_positions(map)
            
            # 2. Draw true map (static obstacles)
            robot.perceived_map.draw(win)
            
            # 3. Draw robot and its perception
            robot.draw(win)
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        robot.move()
                        robot.update_perception()
                    
                    if event.key == pygame.K_r:
                        robot = Robot(sensor_range=5, true_map=map)
        
        pygame.display.update()

if __name__ == "__main__":
    pygame.init()
    main()
    pygame.quit()