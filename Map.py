
import pygame
import numpy as np
import os
from datetime import datetime


class Map:
    def __init__(self, width=800, rows=50):
        '''
        self.grid: 2D array of Spot objects
        self.start: Spot object representing the start point
        self.end: Spot object representing the end point
        self.width: width of the grid
        self.rows: number of rows in the grid
        self.gap: gap between each spot
        
        '''
        
        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)
        self.ORANGE = (255, 165, 0)
        self.TURQUOISE = (64, 224, 208)
        self.PURPLE = (128, 0, 128)
        self.GREY = (128, 128, 128)
        
        self.width = width
        self.rows = rows
        self.gap = width // rows
        self.grid = self.make_grid()
        self.start = None
        self.end = None

    class Spot:
        def __init__(self, row, col, width, total_rows, map_colors):
            self.row = row
            self.col = col
            self.x = col * width
            self.y = row * width
            self.width = width
            self.total_rows = total_rows
            self.neighbors = []
            
            # Use colors from parent Map class
            self.colors = map_colors
            self.color = self.colors['WHITE']

        def get_pos(self):
            return self.row, self.col

        def is_closed(self):
            return self.color == self.colors['RED']

        def is_open(self):
            return self.color == self.colors['GREEN']

        def is_barrier(self):
            return self.color == self.colors['BLACK']

        def is_start(self):
            return self.color == self.colors['ORANGE']

        def is_end(self):
            return self.color == self.colors['TURQUOISE']

        def reset(self):
            self.color = self.colors['WHITE']

        def make_closed(self):
            self.color = self.colors['RED']

        def make_open(self):
            self.color = self.colors['GREEN']

        def make_barrier(self):
            self.color = self.colors['BLACK']

        def make_start(self):
            self.color = self.colors['ORANGE']

        def make_end(self):
            self.color = self.colors['TURQUOISE']

        def make_path(self):
            self.color = self.colors['PURPLE']

        def draw(self, win):
            pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

        def update_neighbors(self, grid):
            self.neighbors = []
            
            # DOWN
            if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
                self.neighbors.append(grid[self.row + 1][self.col])

            # UP
            if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
                self.neighbors.append(grid[self.row - 1][self.col])

            # RIGHT
            if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
                self.neighbors.append(grid[self.row][self.col + 1])

            # LEFT
            if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():
                self.neighbors.append(grid[self.row][self.col - 1])

            # Diagonals
            if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
                self.neighbors.append(grid[self.row + 1][self.col + 1])

            if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
                self.neighbors.append(grid[self.row + 1][self.col - 1])

            if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
                self.neighbors.append(grid[self.row - 1][self.col + 1])

            if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
                self.neighbors.append(grid[self.row - 1][self.col - 1])

        def __lt__(self, other):
            return False

    def make_grid(self):
        grid = []
        # Create color dictionary to pass to Spot
        colors = {
            'WHITE': self.WHITE,
            'BLACK': self.BLACK,
            'RED': self.RED,
            'GREEN': self.GREEN,
            'BLUE': self.BLUE,
            'YELLOW': self.YELLOW,
            'ORANGE': self.ORANGE,
            'TURQUOISE': self.TURQUOISE,
            'PURPLE': self.PURPLE,
            'GREY': self.GREY
        }
        
        for i in range(self.rows):
            grid.append([])
            for j in range(self.rows):
                spot = self.Spot(i, j, self.gap, self.rows, colors)
                grid[i].append(spot)
        return grid
    
    def draw_grid(self, win):
        for i in range(self.rows):
            pygame.draw.line(win, self.GREY, (0, i * self.gap), (self.width, i * self.gap))
            for j in range(self.rows):
                pygame.draw.line(win, self.GREY, (j * self.gap, 0), (j * self.gap, self.width))

    def draw(self, win):
        win.fill(self.WHITE)
        
        for row in self.grid:
            for spot in row:
                spot.draw(win)
                
        self.draw_grid(win)
        pygame.display.update()

    def get_clicked_pos(self, pos):
        x, y = pos
        row = y // self.gap
        col = x // self.gap
        return row, col

    def handle_click(self, pos, button):
        """Handle mouse clicks for placing start, end, and barriers"""
        row, col = self.get_clicked_pos(pos)
        if 0 <= row < self.rows and 0 <= col < self.rows:
            spot = self.grid[row][col]
            
            if button == 1:  # Left click
                if not self.start and spot != self.end:
                    self.start = spot
                    spot.make_start()
                elif not self.end and spot != self.start:
                    self.end = spot
                    spot.make_end()
                elif spot != self.end and spot != self.start:
                    spot.make_barrier()
            
            elif button == 3:  # Right click
                if spot == self.start:
                    self.start = None
                elif spot == self.end:
                    self.end = None
                spot.reset()

    def clear(self):
        """Clear the entire grid"""
        self.grid = self.make_grid()
        self.start = None
        self.end = None
    
    def save_map(self, filename="custom_map"):
        """Save current map to file"""
        grid_array = np.zeros((self.rows, self.rows), dtype=np.int8)
        metadata = {
            'creation_date': str(datetime.now()),
            'grid_size': self.rows,
            'start_pos': None,
            'end_pos': None,
            'barrier_count': 0
        }
        
        for i in range(self.rows):
            for j in range(self.rows):
                spot = self.grid[i][j]
                if spot.is_barrier():
                    grid_array[i][j] = 1
                    metadata['barrier_count'] += 1
                elif spot.is_start():
                    grid_array[i][j] = 2
                    metadata['start_pos'] = (i, j)
                elif spot.is_end():
                    grid_array[i][j] = 3
                    metadata['end_pos'] = (i, j)
        
        save_path = os.path.join("maps", filename)
        os.makedirs("maps", exist_ok=True)
        np.savez(save_path, grid=grid_array, metadata=metadata)
        print(f"Map saved to {save_path}")

    def load_map(self, filename="custom_map.npz"):
        """Load map from file"""
        load_path = os.path.join("maps", filename)
        data = np.load(load_path)
        grid_array = data['grid']
        
        self.clear()  # Reset current grid
        
        for i in range(self.rows):
            for j in range(self.rows):
                if grid_array[i][j] == 1:
                    self.grid[i][j].make_barrier()
                elif grid_array[i][j] == 2:
                    self.grid[i][j].make_start()
                    self.start = self.grid[i][j]
                elif grid_array[i][j] == 3:
                    self.grid[i][j].make_end()
                    self.end = self.grid[i][j]
        

    def update_neighbors(self):
        """Update neighbors for all spots in grid"""
        for row in self.grid:
            for spot in row:
                spot.update_neighbors(self.grid)

'''TESTING ----------------------------------------------------------------'''
def user_draw(width, rows):

    win = pygame.display.set_mode((width, width))  
    pygame.display.set_caption("A* Path Finding Algorithm")
    map_handler = Map(width, rows)
    
    run = True
    while run:
        map_handler.draw(win)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            
            if pygame.mouse.get_pressed()[0]:  # Left click
                pos = pygame.mouse.get_pos()
                map_handler.handle_click(pos, 1)
                
            elif pygame.mouse.get_pressed()[2]:  # Right click
                pos = pygame.mouse.get_pos()
                map_handler.handle_click(pos, 3)
                
            if event.type == pygame.KEYDOWN:
                    
                if event.key == pygame.K_c:
                    map_handler.clear()
                    
                if event.key == pygame.K_s:
                    map_handler.save_map()
                    
                if event.key == pygame.K_l:
                    map_handler.load_map()

    pygame.quit()
    
    
    
if __name__ == "__main__":
    print("Starting map editor...")
    user_draw(800, 50)
