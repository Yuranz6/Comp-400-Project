# Robot Navigation Simulator - Comp 400 Project

A Python-based robot navigation simulator that implements A\* path planning and Artificial Potential Field (APF) for local obstacle avoidance.

## Prerequisites

- Python 3.7+
- PyGame
- NumPy

## Running the Simulator

### Basic Usage

```bash
pip install pygame numpy
```

## Project Structure

```
project/
│
├── Robot.py        # Main simulation file
├── Map.py          # Map handling and visualization
├── Obstacle.py     # Dynamic obstacle implementation
│
└── maps/           # Directory for storing map files
    └── custom_map.npz  # Default map file
```

<code_block_to_apply_changes_from>

```bash
python Robot.py
```

This will start the simulator with default settings and an empty map.

### Loading Existing Maps

```bash
python Robot.py --load_from map_name
```

This loads a map from `maps/map_name.npz`

### Saving Maps

```bash
python Robot.py --save_to map_name
```

Note: Map name must be unique and not already in use.

## Controls

### Map Creation Mode

- **Left Click**: Place walls/obstacles
- **Right Click**: Remove walls/obstacles
- **Ctrl + Left Click**: Set start position (green)
- **Ctrl + Right Click**: Set goal position (red)
- **Ctrl + S**: Save map and initialize robot
- **L**: Load existing map
- **C**: Clear map

### Robot Navigation Mode

- **A**: Toggle auto-navigation
- **Space**: Manual step (when auto-navigation is off)
- **R**: Reset robot to start position
- **C**: Clear map and return to map creation mode

## Features

1. **Map Creation and Editing**

   - Interactive map drawing
   - Save/load map functionality
   - Start and goal position setting

2. **Path Planning**

   - A\* algorithm with direction penalties
   - Automatic waypoint generation
   - Dynamic path visualization

3. **Navigation**

   - Artificial Potential Field (APF) for local navigation
   - Dynamic obstacle avoidance
   - Automatic and manual control modes

4. **Visualization**
   - Real-time path display
   - Robot trajectory tracking
   - Obstacle movement visualization

## Common Issues

1. **Map Loading Errors**

   - Ensure the `maps` directory exists
   - Check file permissions
   - Verify map file format (.npz)

2. **Navigation Issues**
   - Ensure start and goal positions are set
   - Check for path blockage by obstacles
   - Verify sensor range settings

## Notes

- Default map is saved as 'custom_map.npz'
- Sensor range is set to 5 grid cells
- Dynamic obstacles are randomly generated
- Auto-navigation can be interrupted with the 'A' key

```

```
