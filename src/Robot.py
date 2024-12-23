import math
import numpy as np
import pygame 
from Map import Map
from Obstacle import Obstacles
import argparse

class Robot:
    def __init__(self, true_map, sensor_range=3):
        self.current_pos = None
        self.orientation = 0
        self.velocity = 0.8
        self.max_velo = 0.9
        self.min_velo = 0.6
        
        # Map attirbutes
        self.true_map = true_map
        self.perceived_map = Map()
        self.sensor_range = sensor_range
        
        # Path attributes
        self.current_path = []
        self.path_index = 0
        self.goal = None
        
        self.obstacles = Obstacles(num_obstacles=20, map_size=self.true_map.rows)
        
        # Path Planning
        self.path = []
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.has_global_path = False
        
        self.auto_navigation = False
        self.move_delay = 100
        self.last_move_time = 0
        self.update_delay = 50
        self.last_update_time = 0
        self.path_complete = False
        self.replan_delay = 500  # Path replanning delay (milliseconds)
        self.last_replan_time = 0
        
        # Drawing cache
        self.display_surface = pygame.Surface((true_map.width, true_map.width))
        self.static_surface = pygame.Surface((true_map.width, true_map.width))
        self.static_surface.fill((255, 255, 255))
        self.needs_static_redraw = True
        
        self.path_surface = pygame.Surface((true_map.width, true_map.width), pygame.SRCALPHA)
        self.path_surface.fill((255, 255, 255, 0))
        self.path_drawn = False
        
        self.permanent_path_surface = pygame.Surface((true_map.width, true_map.width), pygame.SRCALPHA)
        self.permanent_path_surface.fill((255, 255, 255, 0))
        
        self.trajectory_surface = pygame.Surface((true_map.width, true_map.width), pygame.SRCALPHA)
        self.trajectory_surface.fill((255, 255, 255, 0))
        self.last_trajectory_pos = None
    
    def load_map(self, filename):
        self.perceived_map.load_map(filename = filename)
        
    def update_perception(self):
        current_time = pygame.time.get_ticks()
        if current_time - self.last_update_time >= self.update_delay:
            grid_x = int(round(self.current_pos[0]))
            grid_y = int(round(self.current_pos[1]))
            
            for i in range(self.perceived_map.rows):
                for j in range(self.perceived_map.rows):
                    if not self.true_map.grid[i][j].is_barrier():
                        self.perceived_map.grid[i][j].reset()
                        if self.true_map.grid[i][j].is_start():
                            self.perceived_map.grid[i][j].make_start()
                        elif self.true_map.grid[i][j].is_end():
                            self.perceived_map.grid[i][j].make_end()
            
            # Update static obstacles within sensor range
            for i in range(-self.sensor_range, self.sensor_range + 1):
                for j in range(-self.sensor_range, self.sensor_range + 1):
                    new_row, new_col = grid_x + i, grid_y + j
                    
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
            
            self.needs_static_redraw = True
            self.last_update_time = current_time
    
    # ------------------------A* PATH PLANNING----------------------------------
    # Reference code: https://github.com/Shubhranshu153/Obstacle_Avoidance_MPC_Controller_A_star_Planning
    # with slight modifications
    
    def global_path_planner(self):
        if not self.true_map.end:
            return False
                
            
        # Convert continuous position to grid position for A*
        start = (int(round(self.current_pos[0])), int(round(self.current_pos[1])))
        goal = self.perceived_map.end.get_pos()
        
        open_list = []
        closed_list = set()
        came_from = {}
        
        g_score = {}
        f_score = {}
        parent_dirs = {}
        
        start_node = tuple(start)
        open_list.append(start_node)
        g_score[start_node] = 0
        f_score[start_node] = self.heuristic(start, goal)
        parent_dirs[start_node] = None
        
        directions = [
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]
        
        while open_list:
            # Get node with lowest f_score
            current = min(open_list, key=lambda x: f_score.get(x, float('inf')))
            
            # debug
            print(f"Current: {current}, Goal: {goal}")
            
            if current == tuple(goal):
                print("Path found!")
                path = self.reconstruct_path(came_from, current)
                self.path = path
                self.discretize_path()
                
                # Draw the path
                for i in range(len(self.path) - 1):
                    start = self.path[i]
                    end = self.path[i + 1]
                    if self.is_path_clear(start, end):
                        start_x = start[1] * self.true_map.gap + self.true_map.gap // 2
                        start_y = start[0] * self.true_map.gap + self.true_map.gap // 2
                        end_x = end[1] * self.true_map.gap + self.true_map.gap // 2
                        end_y = end[0] * self.true_map.gap + self.true_map.gap // 2
                        pygame.draw.line(self.permanent_path_surface, (0, 255, 0, 255), 
                                    (start_x, start_y), (end_x, end_y), 2)
                
                self.path_drawn = False
                return True
                
            open_list.remove(current)
            closed_list.add(current)
            
            current_dir = parent_dirs[current]
            
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if neighbor in closed_list:
                    continue
                    
                if not (0 <= neighbor[0] < self.true_map.rows and 
                    0 <= neighbor[1] < self.true_map.rows):
                    continue
                    
                if self.perceived_map.grid[neighbor[0]][neighbor[1]].is_barrier():
                    continue
                
                # Calculate movement cost using helper method
                new_dir = (dx, dy)
                movement_cost = self.calculate_movement_cost(current_dir, new_dir, current, neighbor)
                
                tentative_g = g_score[current] + movement_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    parent_dirs[neighbor] = new_dir
                    if neighbor not in open_list:
                        open_list.append(neighbor)
        
        print("No path found!")
        return False
    def calculate_movement_cost(self, current_dir, new_dir, current_pos, next_pos):
        dx, dy = new_dir
        base_cost = abs(dx) + abs(dy)
        
        direction_penalty = 0
        if current_dir is not None:
            dot_product = current_dir[0]*new_dir[0] + current_dir[1]*new_dir[1]
            norm_current = np.hypot(current_dir[0], current_dir[1])
            norm_new = np.hypot(new_dir[0], new_dir[1])
            if norm_current > 0 and norm_new > 0:
                cos_angle = dot_product / (norm_current * norm_new)
                if cos_angle <= 0:
                    direction_penalty = 4.0
                elif cos_angle <= math.sqrt(2)/2:
                    direction_penalty = 2.0
                else:
                    direction_penalty = 1.0
        
        clearance_score = self.calculate_clearance(next_pos)
        wall_penalty = self.calculate_wall_penalty(next_pos)
        
        total_cost = (
            base_cost * 1.0 +
            direction_penalty * 1.5 +
            wall_penalty * 2.5 +
            (1.0 - clearance_score) * 3.0
        )
        
        return total_cost

    def calculate_clearance(self, pos, radius=3):
        open_cells = 0
        total_cells = 0
        
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                check_x = pos[0] + i
                check_y = pos[1] + j
                
                if (0 <= check_x < self.true_map.rows and 
                    0 <= check_y < self.true_map.rows):
                    total_cells += 1
                    if not self.perceived_map.grid[check_x][check_y].is_barrier():
                        open_cells += 1
        
        return open_cells / total_cells if total_cells > 0 else 0

    def calculate_wall_penalty(self, pos, radius=3):
        penalty = 0
        
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                check_x = pos[0] + i
                check_y = pos[1] + j
                
                if (0 <= check_x < self.true_map.rows and 
                    0 <= check_y < self.true_map.rows):
                    if self.perceived_map.grid[check_x][check_y].is_barrier():
                        dist = np.hypot(i, j)
                        if dist > 0:
                            penalty += 3.0 / (dist * dist)
        
        return penalty

    def heuristic(self, a, b):
        return 0.5 * np.hypot(a[0] - b[0], a[1] - b[1]) + 0.5 *abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def discretize_path(self, spacing=3):
        if not self.path:
            return
        
        self.waypoints = [self.path[0]]
        
        for i in range(1, len(self.path) - 1):
            prev = self.path[i-1]
            curr = self.path[i]
            next_point = self.path[i+1]
            
            v1_x = curr[0] - prev[0]
            v1_y = curr[1] - prev[1]
            
            v2_x = next_point[0] - curr[0]
            v2_y = next_point[1] - curr[1]
            
            is_prev_diagonal = (v1_x != 0 and v1_y != 0)
            is_next_diagonal = (v2_x != 0 and v2_y != 0)
            
            # Add waypoint if:
            # 1. Direction changes significantly OR
            # 2. Movement changes between diagonal and cardinal
            add_waypoint = False
            
            angle_change = abs(np.arctan2(v1_x*v2_y - v1_y*v2_x, v1_x*v2_x + v1_y*v2_y))
            if angle_change > np.pi/4:
                add_waypoint = True
            
            # Check diagonal/cardinal transitions
            if is_prev_diagonal != is_next_diagonal:
                add_waypoint = True
                
            if add_waypoint:
                self.waypoints.append(curr)
                print(f"Added waypoint at {curr} - angle change: {angle_change}, diagonal transition: {is_prev_diagonal != is_next_diagonal}")
            else:
                # Check spacing from last waypoint
                last_waypoint = self.waypoints[-1]
                dist = ((curr[0] - last_waypoint[0])**2 + 
                    (curr[1] - last_waypoint[1])**2)**0.5
                if dist >= spacing:
                    self.waypoints.append(curr)
        
        if self.path[-1] not in self.waypoints:
            self.waypoints.append(self.path[-1])
        
        print('Total waypoints:', len(self.waypoints))
        
    
    # ---------------------------END OF A* PATH PLANNING----------------------------
    
    # ---------------------------(windowed) Artificial Potential Field(AFP) for local replanning----------------
    
    def calculate_window_potential(self):
        """Calculate APF with all force components"""
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints) - 1:
            return None
        
        curr_x, curr_y = self.current_pos
        next_waypoint = self.waypoints[self.current_waypoint_idx + 1]
        
        obstacles_in_range = self.get_obstacles_in_range()
        
        robot_dir = self.calculate_robot_direction(next_waypoint)
        if robot_dir is None:
            return None
        
        f_att_x, f_att_y = self.calculate_attractive_force(next_waypoint)
        f_rep_x, f_rep_y = self.calculate_repulsive_force(obstacles_in_range, robot_dir)
        f_escape_x, f_escape_y = self.calculate_escape_force(obstacles_in_range, robot_dir, next_waypoint)
        
        fx = f_att_x + f_rep_x + 2.0 * f_escape_x
        fy = f_att_y + f_rep_y + 2.0 * f_escape_y 
        
        force_magnitude = np.hypot(fx, fy)
        if force_magnitude > 0:
            fx = fx / force_magnitude
            fy = fy / force_magnitude
        
        return fx, fy, force_magnitude

    def calculate_robot_direction(self, goal):
        """Calculate robot's current direction based on goal"""
        curr_x, curr_y = self.current_pos
        goal_x, goal_y = goal
        
        f_att_x = goal_x - curr_x
        f_att_y = goal_y - curr_y
        
        att_magnitude = np.hypot(f_att_x, f_att_y)
        if att_magnitude > 0:
            return f_att_x / att_magnitude, f_att_y / att_magnitude
        return None

    def calculate_attractive_force(self, goal):
        """Linear attractive force towards goal"""
        alpha = 10.0
        curr_x, curr_y = self.current_pos
        goal_x, goal_y = goal
        
        dist_to_goal = np.hypot(goal_x - curr_x, goal_y - curr_y)
        if dist_to_goal > 0:
            f_att_x = alpha * (goal_x - curr_x) / dist_to_goal
            f_att_y = alpha * (goal_y - curr_y) / dist_to_goal
            return f_att_x, f_att_y
        return 0, 0

    def calculate_repulsive_force(self, obstacles_in_range, robot_dir):
        """Exponential repulsive force from obstacles"""
        beta = 30.0  # Repulsive coefficient
        d0 = 3.0     # Influence threshold
        eta = 2.0    # Decay rate
        curr_x, curr_y = self.current_pos
        f_rep_x = f_rep_y = 0
        
        robot_dir_x, robot_dir_y = robot_dir
        
        for obs_x, obs_y in obstacles_in_range:
            obs_idx = self.obstacles.get_obstacle_positions()[0].index(obs_x)
            obs_vel = self.obstacles.velocities[obs_idx]
            
            dist_to_obs = np.hypot(obs_x - curr_x, obs_y - curr_y)
            if dist_to_obs < d0:
                # Direction from obstacle to robot
                dir_x = (curr_x - obs_x) / dist_to_obs
                dir_y = (curr_y - obs_y) / dist_to_obs
                
                # Check alignment with obstacle
                obs_speed = np.hypot(obs_vel[0], obs_vel[1]) + 1e-6
                obs_dir_x = -obs_vel[0] / obs_speed
                obs_dir_y = -obs_vel[1] / obs_speed
                alignment = robot_dir_x * obs_dir_x + robot_dir_y * obs_dir_y
                
                force_magnitude = beta * np.exp(-eta * (dist_to_obs / d0))
                velocity_factor = 2.0 if alignment > 0 else 1.0
                
                f_rep_x += force_magnitude * dir_x * velocity_factor
                f_rep_y += force_magnitude * dir_y * velocity_factor
        
        return f_rep_x, f_rep_y

    def calculate_escape_force(self, obstacles_in_range, robot_dir, goal):
        """
        Calculate rotational escape force for collision avoidance
        
        Parameters:
        - obstacles_in_range: List of (x,y) positions of nearby obstacles
        - robot_dir: Tuple of (dir_x, dir_y) representing robot's current direction
        - goal: Tuple of (goal_x, goal_y) for the current waypoint
        
        Returns:
        - f_escape_x, f_escape_y: Components of the escape force
        """
        escape_force = 3.0      # Base escape force magnitude
        prediction_steps = 5    # Number of future steps to check
        dt = 0.5               # Time step for prediction
        d0 = 3.0               # Base distance threshold
        
        curr_x, curr_y = self.current_pos
        goal_x, goal_y = goal
        robot_dir_x, robot_dir_y = robot_dir
        f_escape_x = f_escape_y = 0
        
        for obs_x, obs_y in obstacles_in_range:
            obs_idx = self.obstacles.get_obstacle_positions()[0].index(obs_x)
            obs_vel = self.obstacles.velocities[obs_idx]
            
            dist_to_obs = np.hypot(obs_x - curr_x, obs_y - curr_y)
            if dist_to_obs < 0.1:
                continue
                
            # Direction from obstacle to robot
            dir_x = (curr_x - obs_x) / dist_to_obs
            dir_y = (curr_y - obs_y) / dist_to_obs
            
            # Calculate alignment with obstacle
            obs_speed = np.hypot(obs_vel[0], obs_vel[1])
            if obs_speed > 0:
                obs_dir_x = obs_vel[0] / obs_speed
                obs_dir_y = obs_vel[1] / obs_speed
                alignment = abs(robot_dir_x * obs_dir_x + robot_dir_y * obs_dir_y)
            else:
                alignment = 0
                
            # Check future positions for potential collision
            min_future_dist = float('inf')
            for step in range(1, prediction_steps + 1):
                time = step * dt
                pred_robot_x = curr_x + robot_dir_x * self.velocity * time
                pred_robot_y = curr_y + robot_dir_y * self.velocity * time
                pred_obs_x = obs_x + obs_vel[0] * time
                pred_obs_y = obs_y + obs_vel[1] * time
                
                pred_dist = np.hypot(pred_robot_x - pred_obs_x, 
                                pred_robot_y - pred_obs_y)
                min_future_dist = min(min_future_dist, pred_dist)
            
            # Calculate collision risk based on alignment and minimum future distance
            collision_risk = alignment * (1 - min_future_dist/d0)
            collision_risk = max(0, min(1, collision_risk))
            
            if collision_risk > 0:
                # Calculate perpendicular escape directions
                escape_dir_x = -dir_y  # Perpendicular to direction vector
                escape_dir_y = dir_x
                
                escape_pos_1 = (int(round(curr_x + escape_dir_x)), 
                            int(round(curr_y + escape_dir_y)))
                escape_pos_2 = (int(round(curr_x - escape_dir_x)), 
                            int(round(curr_y - escape_dir_y)))
                
                valid_1 = (0 <= escape_pos_1[0] < self.true_map.rows and 
                        0 <= escape_pos_1[1] < self.true_map.rows and
                        not self.perceived_map.grid[escape_pos_1[0]][escape_pos_1[1]].is_barrier())
                valid_2 = (0 <= escape_pos_2[0] < self.true_map.rows and 
                        0 <= escape_pos_2[1] < self.true_map.rows and
                        not self.perceived_map.grid[escape_pos_2[0]][escape_pos_2[1]].is_barrier())
                
                if valid_1 and not valid_2:
                    pass
                elif valid_2 and not valid_1:
                    escape_dir_x = -escape_dir_x
                    escape_dir_y = -escape_dir_y
                elif valid_1 and valid_2:
                    if (escape_dir_x * (goal_x - curr_x) + 
                        escape_dir_y * (goal_y - curr_y)) < 0:
                        escape_dir_x = -escape_dir_x
                        escape_dir_y = -escape_dir_y
                else:
                    # Neither valid, try diagonal escape
                    escape_dir_x = robot_dir_x + (1 if goal_x > curr_x else -1)
                    escape_dir_y = robot_dir_y + (1 if goal_y > curr_y else -1)
                    mag = np.hypot(escape_dir_x, escape_dir_y)
                    if mag > 0:
                        escape_dir_x /= mag
                        escape_dir_y /= mag
                
                escape_scale = escape_force * collision_risk * 1.5
                f_escape_x += escape_scale * escape_dir_x
                f_escape_y += escape_scale * escape_dir_y
        
        return f_escape_x, f_escape_y

    def get_obstacles_in_range(self):
        curr_x, curr_y = self.current_pos
        ox, oy = self.obstacles.get_obstacle_positions()
        return [(x, y) for x, y in zip(ox, oy) 
                if np.hypot(x - curr_x, y - curr_y) <= self.sensor_range]
    
    # ---------------------------END OF LOCAL REPLANNING----------------
    # ---------------------------ROBOT NAVIGATION----------------------------------
    def move(self):
        """Move robot along the planned path while avoiding dynamic obstacles"""
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints) - 1:
            return False
        
        forces = self.calculate_window_potential()
        if forces is None:
            return False
        
        fx, fy, force_magnitude = forces
        
        curr_x, curr_y = self.current_pos
        
        # Adjust velocity based on force magnitude
        velo = self.velocity * abs((force_magnitude / (10.0 - 30.0*len(self.get_obstacles_in_range()) - 2.0 * 3.0)))
        velo_adj = max(self.min_velo, min(self.max_velo, velo))
        
        #debug 
        # print('speed: ', velo_adj, 'force: ', fx, ' ', fy, 'obstacles : ', len(self.get_obstacles_in_range()))

        
        new_x = curr_x + fx * velo_adj
        new_y = curr_y + fy * velo_adj
        
        grid_x = int(round(new_x))
        grid_y = int(round(new_y))
        
        # Check if new position is valid
        if (0 <= grid_x < self.true_map.rows and 
            0 <= grid_y < self.true_map.rows and 
            not self.perceived_map.grid[grid_x][grid_y].is_barrier()):
            
            ox, oy = self.obstacles.get_obstacle_positions()
            dynamic_obstacles = set(zip(ox, oy))
            
            if (grid_x, grid_y) not in dynamic_obstacles:
                self.current_pos = [new_x, new_y]
                self.update_perception()
                
                # Check if reached current waypoint
                next_waypoint = self.waypoints[self.current_waypoint_idx + 1]
                dist_to_waypoint = np.hypot(next_waypoint[0] - grid_x, 
                                        next_waypoint[1] - grid_y)
                
                if dist_to_waypoint < 1.0: 
                    self.current_waypoint_idx += 1
                    if self.current_waypoint_idx >= len(self.waypoints) - 1:
                        return False
                
                return True
        
        return False
    
    def check_path_validity(self):
        """Check if current path is still valid (no obstacles blocking it)"""
        if not self.path:
            return False
        
        current_idx = self.current_waypoint_idx
        if current_idx >= len(self.waypoints) - 1:
            return True
        
        next_waypoint = self.waypoints[current_idx + 1]
        current = self.current_pos
        
        # Simple line-of-sight check with wider corridor
        dx = next_waypoint[0] - current[0]
        dy = next_waypoint[1] - current[1]
        steps = max(abs(dx), abs(dy))
        
        if steps == 0:
            return True
        
        x_step = dx/steps
        y_step = dy/steps
        
        # Check corridor area around path
        corridor_width = 2
        
        ox, oy = self.obstacles.get_obstacle_positions()
        obstacle_positions = set((x, y) for x, y in zip(ox, oy))
        
        for i in range(1, int(steps)):
            center_x = int(round(current[0] + i*x_step))
            center_y = int(round(current[1] + i*y_step))
            
            # Check if corridor area has obstacles
            blocked = True
            for dx in range(-corridor_width, corridor_width + 1):
                for dy in range(-corridor_width, corridor_width + 1):
                    check_x = center_x + dx
                    check_y = center_y + dy
                    
                    if (0 <= check_x < self.true_map.rows and 
                        0 <= check_y < self.true_map.rows and 
                        not self.perceived_map.grid[check_x][check_y].is_barrier() and
                        (check_x, check_y) not in obstacle_positions):  # Check dynamic obstacles
                        blocked = False
                        break
                if not blocked:
                    break
                
            if blocked:
                return False
        
        return True
    
    def draw(self, surface):
        """Draw robot and environment with continuous position support"""
        # Draw static elements 
        if self.needs_static_redraw:
            self.static_surface.fill((255, 255, 255))
            self.perceived_map.draw(self.static_surface)
            self.needs_static_redraw = False
        
        self.display_surface.blit(self.static_surface, (0, 0))
        
        self.display_surface.blit(self.permanent_path_surface, (0, 0))
        
        # Draw current path if exists and not drawn
        if not self.path_drawn and self.path:
            self.path_surface.fill((255, 255, 255, 0))  # Clear with transparency
            for i in range(len(self.path) - 1):
                start = self.path[i]
                end = self.path[i + 1]
                start_x = start[1] * self.true_map.gap + self.true_map.gap // 2
                start_y = start[0] * self.true_map.gap + self.true_map.gap // 2
                end_x = end[1] * self.true_map.gap + self.true_map.gap // 2
                end_y = end[0] * self.true_map.gap + self.true_map.gap // 2
                pygame.draw.line(self.path_surface, (0, 255, 0, 128), 
                            (start_x, start_y), (end_x, end_y), 2)
            self.path_drawn = True
        
        # Draw path surface
        self.display_surface.blit(self.path_surface, (0, 0))
        
        # Draw waypoints 
        if self.waypoints:
            for i, waypoint in enumerate(self.waypoints):
                if i > self.current_waypoint_idx:  # Only draw upcoming waypoints
                    wp_x = waypoint[1] * self.true_map.gap + self.true_map.gap // 2
                    wp_y = waypoint[0] * self.true_map.gap + self.true_map.gap // 2
                    # Draw waypoint as small yellow circle
                    pygame.draw.circle(self.display_surface, (255, 255, 0), 
                                    (wp_x, wp_y), self.true_map.gap // 6)
        
        # Draw dynamic obstacles
        ox, oy = self.obstacles.get_obstacle_positions()
        cell_size = self.true_map.gap
        for x, y in zip(ox, oy):
            # Convert obstacle grid positions to screen coordinates
            obs_x = y * cell_size + cell_size // 2
            obs_y = x * cell_size + cell_size // 2
            # Draw obstacle
            pygame.draw.circle(self.display_surface, (255, 165, 0),
                            (obs_x, obs_y), cell_size // 3)
            
        
        # Draw robot at continuous position
        # Convert continuous position to screen coordinates
        screen_x = self.current_pos[1] * self.true_map.gap + self.true_map.gap // 2
        screen_y = self.current_pos[0] * self.true_map.gap + self.true_map.gap // 2
        
        # Draw robot body
        pygame.draw.circle(self.display_surface, (0, 0, 255), 
                        (int(screen_x), int(screen_y)), 
                        self.true_map.gap // 3)
        
        # Draw robot orientation indicator (optional)
        orientation_length = self.true_map.gap // 2
        end_x = screen_x + orientation_length * np.cos(self.orientation)
        end_y = screen_y + orientation_length * np.sin(self.orientation)
        pygame.draw.line(self.display_surface, (0, 0, 0),
                        (int(screen_x), int(screen_y)), 
                        (int(end_x), int(end_y)), 2)
        
        # Draw sensor range indicator (optional)
        sensor_radius = self.sensor_range * self.true_map.gap
        pygame.draw.circle(self.display_surface, (200, 200, 200), 
                        (int(screen_x), int(screen_y)), 
                        int(sensor_radius), 1)
        
        
        # Blit final surface
        surface.blit(self.display_surface, (0, 0))
    
    # ---------------------------Auto NAVIGATION----------------------------
    def auto_navigate(self, current_time):
        """Smart auto-navigation logic"""
        if not self.auto_navigation:
            return False
        
        goal_pos = self.true_map.end.get_pos()
        dist_to_goal = np.hypot(
            self.current_pos[0] - goal_pos[0],
            self.current_pos[1] - goal_pos[1]
        )
        
        if dist_to_goal < 0.5:  
            print("Goal reached!")
            self.path_complete = True
            self.auto_navigation = False
            return False
        
        # Execute movement
        if current_time - self.last_move_time >= self.move_delay:
            success = self.move()
            if success:
                self.last_move_time = current_time
                
                if self.current_waypoint_idx < len(self.waypoints) - 1:
                    next_waypoint = self.waypoints[self.current_waypoint_idx + 1]
                    if self.current_pos == next_waypoint:
                        self.current_waypoint_idx += 1
                        print(f"Reached waypoint {self.current_waypoint_idx}")
            else:
                print("Oh no movement possible, robot need to stop and think! ")
        
        return True

    def toggle_auto_navigation(self):
        """Toggle auto/manual navigation mode"""
        if not self.auto_navigation:  
            print("Enabling auto-navigation...")
            self.path_complete = False
            self.current_waypoint_idx = 0
            
            if not self.has_global_path:
                print("Planning initial global path...")
                if not self.global_path_planner():
                    print("Failed to find initial path")
                    return False
                self.has_global_path = True
                print("Initial path found")
            
            self.auto_navigation = True
            print("Auto-navigation enabled")
            return True
        else:  # Turning off auto-navigation
            self.auto_navigation = False
            print("Auto-navigation disabled")
            return False
    
    
    def is_path_clear(self, start, end):
        """Check if there is an obstacle between two points"""
        x1, y1 = start
        x2, y2 = end
        
        dx = x2 - x1
        dy = y2 - y1
        steps = max(abs(dx), abs(dy))
        
        if steps == 0:
            return True
        
        x_step = dx / steps
        y_step = dy / steps
        
        # Check more points along the path
        for i in range(int(steps + 1)):
            x = int(round(x1 + i * x_step))
            y = int(round(y1 + i * y_step))
            
            # Check a small area around the point
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    check_x = x + dx
                    check_y = y + dy
                    if (not (0 <= check_x < self.true_map.rows and 
                            0 <= check_y < self.true_map.rows) or
                        self.perceived_map.grid[check_x][check_y].is_barrier()):
                        return False
        
        return True

'''

TESTING ----------------------------------------------------------------


'''
def main():
    parser = argparse.ArgumentParser(description='Robot Navigation Simulator')
    parser.add_argument('--load_from', type=str, help='Load map from maps/[map_name].npz')
    parser.add_argument('--save_to', type=str, help='Save map to maps/[map_name].npz')
    args = parser.parse_args()

    pygame.init()
    font = pygame.font.SysFont('arial', 20)
    width = 800
    map = Map(width)
    map_drawn = False
    robot = None

    pygame.display.set_mode((width, width), pygame.DOUBLEBUF | pygame.HWSURFACE)
    win = pygame.display.get_surface()
    clock = pygame.time.Clock()
    obstacle_update_delay = 100
    last_obstacle_update = 0

    def initialize_robot(load_path='../maps/custom_map.npz'):
        nonlocal robot, map_drawn
        try:
            robot = Robot(true_map=map, sensor_range=5)
            robot.load_map(filename=load_path)
            robot.current_pos = map.start.get_pos()
            robot.update_perception()
            robot.global_path_planner()
            map_drawn = True
            print(f'Map loaded from {load_path}')
            return True
        except Exception as e:
            print(f"Initialization error: {e}")
            map_drawn = False
            robot = None
            return False

    # Initial map load 
    if args.load_from:
        load_path = f'maps/{args.load_from}.npz'
        map.load_map(filename=load_path)
        initialize_robot(load_path)

    running = True
    while running:
        clock.tick(60)
        current_time = pygame.time.get_ticks()
        
        if not map_drawn:
            # Map creation mode
            map.draw(win)
            instruction_text = font.render("L: Load map   Ctrl+s: Save map   A: Auto navigation", True, (0, 0, 0))
            text_rect = instruction_text.get_rect()
            text_rect.bottomleft = (10, width - 10)
            win.blit(instruction_text, text_rect)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if pygame.mouse.get_pressed()[0]:
                    map.handle_click(pygame.mouse.get_pos(), 1)
                elif pygame.mouse.get_pressed()[2]:
                    map.handle_click(pygame.mouse.get_pos(), 3)
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_l:
                        load_path = f'../maps/{args.load_from}.npz' if args.load_from else '../maps/custom_map.npz'
                        map.load_map(filename=load_path)
                        initialize_robot(load_path)
                    elif event.key == pygame.K_s and pygame.key.get_mods() & pygame.KMOD_CTRL and map.start and map.end:
                        save_path = f'../maps/{args.save_to}.npz' if args.save_to else '../maps/custom_map.npz'
                        map.save_map(filename=save_path)
                        print(f'Map saved to {save_path}')
                        initialize_robot(save_path)
                    elif event.key == pygame.K_c:
                        map.clear()
                        map_drawn = False
                        robot = None
        else:
            # Robot navigation mode
            if robot is None:
                map_drawn = False
                continue
                
            if current_time - last_obstacle_update >= obstacle_update_delay:
                robot.obstacles.update_positions(map)
                last_obstacle_update = current_time
            
            if robot.auto_navigation and not robot.auto_navigate(current_time):
                print("Auto-navigation stopped")
            
            robot.draw(win)
            instruction_text = font.render("L: Load map   Ctrl+s: Save map   A: Auto navigation", True, (0, 0, 0))
            text_rect = instruction_text.get_rect()
            text_rect.bottomleft = (10, width - 10)
            win.blit(instruction_text, text_rect)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE and not robot.auto_navigation:
                        robot.move()
                        robot.update_perception()
                    elif event.key == pygame.K_r:
                        load_path = f'../maps/{args.load_from}.npz' if args.load_from else '../maps/custom_map.npz'
                        initialize_robot(load_path)
                    elif event.key == pygame.K_a:
                        if robot.toggle_auto_navigation():
                            print("Auto-navigation started")
                        else:
                            print("Auto-navigation stopped")
                    elif event.key == pygame.K_c:
                        map.clear()
                        map_drawn = False
                        robot = None

        pygame.display.flip()

if __name__ == "__main__":
    main()
    pygame.quit()

