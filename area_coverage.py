#!/usr/bin/env python3
"""
UAV Coverage Path Planner

This module implements the Boustrophedon decomposition algorithm for 
generating efficient coverage paths for unmanned aerial vehicles (UAVs).

The algorithm generates parallel sweep lines across a polygonal area,
optimizing coverage for surveillance, mapping, or spraying applications.

Author: qzhou711
Date: 2021-06-02
"""

from shapely.geometry import Point, Polygon, LineString
from shapely.geometry import MultiLineString, MultiPoint, GeometryCollection
from shapely.errors import TopologicalError
import numpy as np
import matplotlib.pyplot as plt
from logging import error
from math import atan
from typing import List, Tuple, Optional


# =============================================================================
# Rotation Transform Class
# =============================================================================

class RotationTransform:
    """
    Represents a 2D rotation transformation.
    
    Used to rotate polygon coordinates for optimizing coverage path direction.
    The rotation angle is computed based on the polygon's longest edge
    to minimize the number of required coverage passes.
    
    Attributes:
        angle: Rotation angle in degrees
        w: Internal angle in radians (for matrix computation)
        irm: 3x3 rotation matrix
    """
    
    def __init__(self, angle: float):
        """
        Initialize rotation transform with specified angle.
        
        Args:
            angle: Rotation angle in degrees
        """
        self.angle = angle
        # Convert to radians (90-degree offset for coordinate system alignment)
        self.w = np.radians(90 - angle)
        
        # Build 3x3 rotation matrix for 2D transformation
        self.irm = np.mat([
            [np.cos(self.w), -np.sin(self.w), 0.0],
            [np.sin(self.w),  np.cos(self.w), 0.0],
            [0.0,             0.0,             1.0]
        ], dtype='float')
    
    def __repr__(self) -> str:
        return f"RotationTransform(angle={self.angle:.1f}°)"


# =============================================================================
# Main Area Polygon Class
# =============================================================================

class AreaPolygon:
    """
    Represents a polygon area for coverage path planning.
    
    This class implements the Boustrophedon decomposition algorithm,
    which divides a polygonal area into cells and generates efficient
    parallel sweep paths for complete coverage.
    
    The algorithm:
    1. Computes optimal path direction based on longest edge
    2. Rotates polygon to align with path direction
    3. Generates parallel sweep lines across the area
    4. Orders the sweep lines to minimize travel distance
    5. Returns the final coverage path
    
    Attributes:
        P: Original polygon (shapely Polygon)
        rP: Rotated polygon
        rtf: Rotation transform
        origin: Starting point for coverage path
        ft: Path spacing (swath width)
    """
    
    def __init__(
        self,
        coordinates: List[Tuple[float, float]],
        initial_pos: Tuple[float, float],
        interior: List[List[Tuple[float, float]]] = None,
        ft: float = 1.0,
        angle: Optional[float] = None
    ):
        """
        Initialize the area polygon for coverage planning.
        
        Args:
            coordinates: List of (x, y) tuples defining outer boundary
            initial_pos: Initial UAV position (x, y)
            interior: List of holes (each hole is a list of (x, y) tuples)
            ft: Path spacing (swath width between parallel lines)
            angle: Fixed path angle in degrees (None = auto-compute)
        """
        # Initialize polygon with optional holes
        interior = interior or []
        self.P = Polygon(coordinates, interior)
        
        # Compute rotation transform
        if angle:
            # Use provided angle
            self.rtf = RotationTransform(angle)
        else:
            # Auto-compute based on longest edge
            self.rtf = self._compute_longest_edge_transform()
        
        # Rotate polygon for path planning
        self.rP = self._rotate_polygon()
        
        # Find optimal starting point (closest vertex to initial position)
        self.origin = self._get_closest_vertex(self.P.exterior.coords, initial_pos)[0]
        print(f"[INFO] Starting point: {self.origin}")
        
        # Store path spacing
        self.ft = ft
    
    # -------------------------------------------------------------------------
    # Rotation Transform Methods
    # -------------------------------------------------------------------------
    
    def _compute_longest_edge_transform(self) -> RotationTransform:
        """
        Compute rotation angle based on polygon's longest edge.
        
        This optimization minimizes the number of required coverage passes
        by aligning the sweep direction with the polygon's longest dimension.
        
        Returns:
            RotationTransform object
        """
        coords = list(self.P.exterior.coords)
        n = len(coords)
        
        # Calculate edge lengths
        edge_lengths = [
            Point(coords[i]).distance(Point(coords[(i + 1) % n]))
            for i in range(n)
        ]
        
        # Find longest edge
        max_idx = edge_lengths.index(max(edge_lengths))
        print(f"[INFO] Longest edge at index: {max_idx}")
        
        # Compute angle from edge vector
        p1 = coords[max_idx]
        p2 = coords[(max_idx + 1) % n]
        dy = float(p2[1] - p1[1])
        dx = float(p2[0] - p1[0])
        
        angle = np.degrees(np.arctan([dy / dx]))
        return RotationTransform(angle)
    
    def _rotate_points(self, points: np.ndarray) -> np.ndarray:
        """
        Apply rotation transform to a set of points.
        
        Args:
            points: Numpy array of shape (N, 2) containing (x, y) coordinates
            
        Returns:
            Rotated points as numpy array
        """
        rotated = []
        for point in points:
            # Convert to column vector
            point_mat = np.mat([[point[0]], [point[1]], [0]], dtype='float64')
            
            # Apply rotation matrix
            new_point = self.rtf.irm * point_mat
            
            # Extract 2D coordinates
            rotated.append(np.array(new_point[:-1].T, dtype='float64'))
        
        return np.squeeze(np.array(rotated, dtype='float64'))
    
    def _rotate_from(self, points: np.ndarray) -> np.ndarray:
        """
        Rotate points in reverse (apply inverse transform).
        
        Args:
            points: Numpy array of shape (N, 2)
            
        Returns:
            Rotated points in original coordinate system
        """
        if not isinstance(points, np.ndarray):
            raise TypeError("rotate_from requires numpy.ndarray input")
        
        rotated = []
        for point in points:
            point_mat = np.mat([[point[0]], [point[1]], [0]], dtype='float64')
            
            # Apply inverse rotation matrix
            new_point = self.rtf.irm.I * point_mat
            rotated.append(np.array(new_point[:-1].T, dtype='float64'))
        
        return np.squeeze(np.array(rotated, dtype='float64'))
    
    def _rotate_polygon(self) -> Polygon:
        """
        Apply rotation transform to the polygon and its holes.
        
        Returns:
            Rotated Shapely Polygon
        """
        # Rotate exterior boundary
        exterior_points = np.array(self.P.exterior)
        rotated_exterior = self._rotate_points(exterior_points)
        
        # Rotate holes (if any)
        rotated_holes = []
        for hole in self.P.interiors:
            rotated_holes.append(self._rotate_points(np.array(hole)))
        
        return self._arrays_to_polygon(rotated_exterior, rotated_holes)
    
    def _arrays_to_polygon(
        self,
        exterior: np.ndarray,
        holes: List[np.ndarray]
    ) -> Polygon:
        """
        Convert numpy arrays to Shapely Polygon.
        
        Args:
            exterior: Numpy array of exterior coordinates
            holes: List of numpy arrays for hole coordinates
            
        Returns:
            Shapely Polygon object
        """
        new_exterior = [(float(p[0]), float(p[1])) for p in exterior]
        new_interiors = []
        
        for hole in holes:
            new_holes = [(float(p[0]), float(p[1])) for p in hole]
            new_interiors.append(new_holes)
        
        return Polygon(new_exterior, new_interiors)
    
    # -------------------------------------------------------------------------
    # Path Generation Methods
    # -------------------------------------------------------------------------
    
    def _generate_sweep_lines(self) -> List[LineString]:
        """
        Generate parallel sweep lines across the rotated polygon.
        
        Creates equally-spaced parallel lines that cover the entire
        polygon area. The spacing is determined by the ft parameter.
        
        Returns:
            List of Shapely LineStrings representing sweep lines
        """
        # Get bounding box of rotated polygon
        min_x, min_y, max_x, max_y = self.rP.bounds
        
        # Create initial sweep line from bottom of bounding box
        start_point = (min_x, min_y)
        end_point = (min_x, max_y)
        sweep_line = LineString([start_point, end_point])
        
        # Find intersection with polygon
        try:
            initial_line = self.rP.intersection(sweep_line)
        except TopologicalError:
            error("[ERROR] Failed to compute initial intersection")
            return []
        
        lines = [initial_line]
        
        # Generate parallel lines at specified spacing
        num_lines = int((max_x - min_x) / self.ft) + 2
        
        for i in range(1, num_lines):
            # Offset line to the right
            offset_line = sweep_line.parallel_offset(i * self.ft, 'right')
            
            # Check if line intersects polygon
            if not self.rP.intersects(offset_line):
                continue
            
            try:
                # Compute exact intersection
                intersection = self.rP.intersection(offset_line)
                
                # Handle different geometry types
                if isinstance(intersection, GeometryCollection):
                    continue
                elif isinstance(intersection, MultiLineString):
                    for ln in intersection.geoms:
                        lines.append(ln)
                else:
                    lines.append(intersection)
                    
            except TopologicalError:
                error(f"[ERROR] Intersection failed at line {i}")
                continue
        
        return lines
    
    def _get_closest_vertex(
        self,
        vertices: List[Tuple[float, float]],
        reference: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """
        Sort vertices by distance to a reference point.
        
        Args:
            vertices: List of (x, y) coordinate tuples
            reference: Reference point (x, y)
            
        Returns:
            Sorted list of vertices
        """
        ref_point = Point(*reference)
        return sorted(vertices, key=lambda v: ref_point.distance(Point(*v)))
    
    def _order_sweep_lines(
        self,
        lines: List[LineString],
        start_point: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """
        Order sweep lines to create efficient coverage path.
        
        Uses a nearest-neighbor approach to minimize travel distance
        between consecutive coverage segments.
        
        Args:
            lines: List of sweep line geometries
            start_point: Starting position (x, y)
            
        Returns:
            Ordered list of waypoints
        """
        waypoints = []
        current_pos = start_point
        
        while lines:
            # Sort lines by distance to current position
            self._sort_by_distance(lines, current_pos)
            
            # Get next line
            if not lines:
                break
                
            line = lines.pop(0)
            
            # Skip invalid geometries
            if isinstance(line, (GeometryCollection, Point, MultiPoint)):
                continue
            
            # Handle multi-line geometries
            if isinstance(line, MultiLineString):
                for ln in line.geoms:
                    lines.append(ln)
                continue
            
            # Get endpoints of line segment
            xs, ys = line.xy
            endpoints = list(zip(xs, ys))
            
            # Find endpoint farthest from current position (determines direction)
            direction_point = self._get_closest_vertex(endpoints, current_pos)[0]
            
            # Check for obstacles (holes)
            current_line = LineString([current_pos, direction_point])
            for hole in self.rP.interiors:
                if isinstance(hole.intersection(current_line), LineString):
                    print(f"[WARNING] Path intersects hole, skipping segment")
            
            # Add waypoints
            waypoints.append(current_pos)
            waypoints.append(direction_point)
            
            # Update current position to end of segment
            current_pos = self._get_closest_vertex(endpoints, direction_point)[1]
        
        return waypoints
    
    def _sort_by_distance(
        self,
        lines: List[LineString],
        reference: Tuple[float, float]
    ):
        """
        Sort a list of lines by their distance to a reference point.
        
        In-place sorting operation.
        
        Args:
            lines: List of LineString objects
            reference: Reference point (x, y)
        """
        lines.sort(key=lambda ln: ln.distance(Point(*reference)))
    
    # -------------------------------------------------------------------------
    # Public Methods
    # -------------------------------------------------------------------------
    
    def generate_coverage_path(self, origin: Tuple[float, float] = None) -> LineString:
        """
        Generate the complete coverage path for this polygon.
        
        This is the main public method that orchestrates the entire
        path planning algorithm.
        
        Args:
            origin: Optional starting point (x, y). If None, uses 
                   the vertex closest to initial position.
            
        Returns:
            Shapely LineString representing the coverage path
        """
        # Handle origin rotation
        if origin:
            # Convert origin to rotated coordinate system
            origin = self._rotate_points(np.array([origin])).tolist()[0]
        else:
            # Use pre-computed optimal origin
            origin = self._rotate_points(np.array([self.origin])).tolist()[0]
        
        # Generate sweep lines
        sweep_lines = self._generate_sweep_lines()
        
        # Order sweep lines for efficient coverage
        waypoints = self._order_sweep_lines(sweep_lines, origin)
        
        # Convert to numpy array
        waypoint_array = np.array(waypoints, dtype='float')
        
        # Rotate back to original coordinate system
        final_path = self._rotate_from(waypoint_array)
        
        return LineString(final_path)


# =============================================================================
# Visualization Utilities
# =============================================================================

def plot_coordinates(ax, geometry):
    """Plot point coordinates as markers."""
    x, y = geometry.xy
    ax.plot(x, y, 'o', color='#999999', zorder=1)


def plot_boundary(ax, geometry):
    """Plot boundary points."""
    x, y = zip(*[(p.x, p.y) for p in geometry.boundary])
    ax.plot(x, y, 'o', color='#000000', zorder=1)


def plot_path(ax, geometry, color='blue', linewidth=3):
    """Plot a path/line with styling."""
    x, y = geometry.xy
    ax.plot(
        x, y,
        alpha=0.7,
        linewidth=linewidth,
        solid_capstyle='round',
        color=color,
        zorder=2
    )


# =============================================================================
# Main Execution
# =============================================================================

if __name__ == '__main__':
    print("[UAV Coverage Path Planner]")
    print("=" * 50)
    
    # Define polygon with hole
    # Exterior boundary (pentagon-like shape)
    exterior = [(0, 0), (4, 4), (0, 8), (-4, 4), (-9, 3)]
    
    # Interior hole (optional)
    holes = [[(0, 0), (0, 0), (0, 0), (0, 0)]]  # Empty hole
    
    # Create area polygon for coverage planning
    # Parameters:
    #   - exterior: polygon boundary coordinates
    #   - (-5, 10): initial UAV position
    #   - ft=0.5: path spacing (swath width)
    #   - angle=30: fixed path angle in degrees
    polygon = AreaPolygon(
        exterior,
        (-5, 10),
        interior=holes,
        ft=0.5,
        angle=30
    )
    
    print(f"[INFO] Rotation angle: {polygon.rtf.angle:.1f}°")
    
    # Generate coverage path starting from origin (0, 0)
    coverage_path = polygon.generate_coverage_path(origin=(0.0, 0.0))
    
    # Visualize results
    fig = plt.figure(1, dpi=90)
    ax = fig.add_subplot(121)
    
    # Plot rotated polygon (coverage frame)
    plt.plot(*polygon.rP.exterior.xy, 'b-', linewidth=2, label='Coverage Area')
    
    # Plot coverage path
    plot_coordinates(ax, coverage_path)
    plot_boundary(ax, coverage_path)
    plot_path(ax, coverage_path, color='red')
    
    # Plot original polygon
    plt.plot(*polygon.P.exterior.xy, 'g--', linewidth=1, label='Original')
    
    # Set equal aspect ratio
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.title("UAV Coverage Path Planning\n(Boustrophedon Decomposition)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    
    plt.tight_layout()
    plt.show()
    
    print("[INFO] Path generation complete!")
    print(f"[INFO] Path contains {len(list(coverage_path.coords))} waypoints")
