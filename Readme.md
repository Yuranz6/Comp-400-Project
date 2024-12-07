# Robot Navigation Simulator - Project with Dr.Joseph Vybihal

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

<code_block_to_apply_changes_from>

```bash
python Robot.py
```

This will start the simulator with default settings and an empty map.

### Loading Existing Maps

```bash
python Robot.py --load_from map_name
```

This automatically loads a map from `maps/map_name.npz`

### Saving Maps

```bash
python Robot.py --save_to map_name
```

This command will save user-created map under the name specified by `map_name` in the `maps` directory. (after pressing 'ctrl + s' in the map creation mode)

Note: Map name must be unique and not already in use.

## Controls

### Map Creation Mode

- **Left Click**: Place walls/obstacles
- **Right Click**: Remove walls/obstacles
- **Ctrl + Left Click**: Set start position (green)
- **Ctrl + Right Click**: Set goal position (blue)
- **Ctrl + S**: Save map and initialize robot
- **L**: Load existing map
- **C**: Clear map

### Robot Navigation Mode

- **A**: Toggle auto-navigation
- **Space**: Manual step (when auto-navigation is off)
- **R**: Reset robot to start position
- **C**: Clear map and return to map creation mode

## Common Issues

1. **Map Loading Errors**

   - Ensure the `maps` directory exists
   - Check file permissions

2. **Navigation Issues**
   - Ensure start and goal positions are set

## Notes

- Default map is saved as 'custom_map.npz'
- Dynamic obstacles are randomly generated
- Auto-navigation can be interrupted with the 'A' key
- The 3 map configurations used in the experiments are saved in the 'maps' directory (config_1.npz, config_2.npz, config_3.npz)
  to reproduce the experiments, just run the following:

```bash
python Robot.py --load_from config_number
```

there are also several other maps in the 'maps' directory, feel free to test on those as well!

```

```
