# UAV-Coverage-Planner

🤖 **UAV Coverage Path Planner** - Autonomous drone path planning using Boustrophedon decomposition

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## 📋 Overview

This project implements the **Boustrophedon decomposition algorithm** for generating efficient coverage paths for unmanned aerial vehicles (UAVs). The algorithm is widely used in agricultural spraying, aerial surveying, surveillance, and search-and-rescue operations.

### What is Boustrophedon Decomposition?

Boustrophedon (Greek: "ox-turning") decomposition divides a polygonal area into cells and generates parallel sweep lines that cover the entire area with minimal travel distance. The term comes from the way oxen plow fields - moving in parallel lines, then turning at the end to cover the next strip.

### Key Features

- 📐 **Polygon Support**: Handles simple polygons and polygons with holes
- 🔄 **Auto-Rotation**: Computes optimal path angle based on longest edge
- 📏 **Configurable Spacing**: Adjustable swath width (path spacing)
- 🎯 **Efficient Ordering**: Nearest-neighbor path ordering minimizes travel
- 📊 **Visualization**: Built-in plotting utilities for path visualization

## 🚀 Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/qzhou711/UAV-Coverage-Planner.git
cd UAV-Coverage-Planner

# Install dependencies
pip install numpy shapely matplotlib
```

### Basic Usage

```python
from area_coverage import AreaPolygon, plot_coordinates, plot_boundary, plot_path
import matplotlib.pyplot as plt

# Define polygon boundary (pentagon example)
exterior = [(0, 0), (4, 4), (0, 8), (-4, 4), (-9, 3)]

# Create area polygon for coverage planning
polygon = AreaPolygon(
    coordinates=exterior,
    initial_pos=(-5, 10),  # Starting UAV position
    ft=0.5,                # Path spacing (swath width)
    angle=30               # Path angle in degrees (None = auto-compute)
)

# Generate coverage path
path = polygon.generate_coverage_path(origin=(0.0, 0.0))

# Visualize
fig, ax = plt.subplots()
plt.plot(*polygon.rP.exterior.xy, 'b-', label='Coverage Area')
plot_path(ax, path, color='red')
plt.plot(*polygon.P.exterior.xy, 'g--', label='Original')
plt.legend()
plt.gca().set_aspect('equal')
plt.show()
```

### Command Line

```bash
# Run the example script
python area_coverage.py
```

## 📁 Project Structure

```
UAV-Coverage-Planner/
├── area_coverage.py      # Main implementation
├── README.md             # This file
└── LICENSE               # License file
```

## 🔧 API Reference

### AreaPolygon Class

```python
AreaPolygon(
    coordinates: List[Tuple[float, float]],  # Outer boundary vertices
    initial_pos: Tuple[float, float],         # Starting position
    interior: List[List[Tuple[float, float]]] = None,  # Holes
    ft: float = 1.0,                          # Path spacing
    angle: Optional[float] = None             # Path angle (None = auto)
)
```

#### Methods

- `generate_coverage_path(origin=None)` - Generate the complete coverage path

#### Properties

- `rtf.angle` - Computed rotation angle in degrees
- `P` - Original polygon (Shapely)
- `rP` - Rotated polygon (Shapely)

### Visualization Functions

```python
plot_coordinates(ax, geometry)  # Plot point markers
plot_boundary(ax, geometry)     # Plot boundary points
plot_path(ax, geometry, color='red', linewidth=3)  # Plot path line
```

## 📊 Algorithm Details

### How It Works

1. **Polygon Input**: Define the coverage area as a polygon (with optional holes)
2. **Angle Optimization**: Compute the optimal sweep direction based on the longest edge
3. **Rotation**: Rotate the polygon to align with the sweep direction
4. **Sweep Generation**: Create parallel lines at specified spacing across the polygon
5. **Path Ordering**: Order the sweep lines using nearest-neighbor to minimize travel
6. **Coordinate Transform**: Rotate the path back to original coordinate system

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `coordinates` | List[Tuple] | Required | Polygon vertices (CCW order) |
| `initial_pos` | Tuple | Required | Starting UAV position |
| `interior` | List[List] | None | List of hole polygons |
| `ft` | float | 1.0 | Path spacing (swath width) |
| `angle` | float/None | None | Fixed angle, or auto-compute |

### Output

The `generate_coverage_path()` method returns a **Shapely LineString** containing the complete coverage path as a sequence of waypoints.

```python
# Access waypoints
path = polygon.generate_coverage_path()
for i, (x, y) in enumerate(path.coords):
    print(f"Waypoint {i}: ({x:.2f}, {y:.2f})")

# Path length
path_length = path.length
print(f"Total path length: {path_length:.2f} meters")
```

## 🧪 Examples

### Example 1: Simple Polygon

```python
# Simple rectangular area
exterior = [(0, 0), (10, 0), (10, 5), (0, 5)]
polygon = AreaPolygon(exterior, (-1, 2), ft=1.0)
path = polygon.generate_coverage_path()
```

### Example 2: Polygon with Hole

```python
# Polygon with a hole (obstacle in the middle)
exterior = [(0, 0), (10, 0), (10, 10), (0, 10)]
hole = [(4, 4), (6, 4), (6, 6), (4, 6)]  # Square hole

polygon = AreaPolygon(
    exterior,
    (-1, 5),
    interior=[hole],
    ft=0.5
)
path = polygon.generate_coverage_path()
```

### Example 3: Auto-Angle Computation

```python
# Let the algorithm compute optimal angle based on longest edge
polygon = AreaPolygon(
    exterior,
    initial_pos,
    ft=1.0,
    angle=None  # Auto-compute
)
print(f"Optimal angle: {polygon.rtf.angle:.1f}°")
```

## 📈 Performance

The algorithm runs in O(n log n) time complexity, where n is the number of polygon vertices, primarily due to the sorting operations in path ordering.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## 📧 Contact

For questions or suggestions, please open an issue on GitHub.

## 📚 References

1. Choset, H. (2001). Coverage of Known Spaces: The Boustrophedon Cellular Decomposition. Springer.
2. Acar, E.U., Choset, H., Rizzi, A.A., Atkar, P.N., and Hull, D. (2002). Exact cellular decompositions in terms of critical points of Morse functions. IEEE International Conference on Robotics and Automation.

---

**Note**: This implementation is designed for 2D path planning. For 3D applications (e.g., terrain following), additional processing would be required.
