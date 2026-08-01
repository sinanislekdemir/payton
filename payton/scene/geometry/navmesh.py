"""Navigation mesh for pathfinding on 3D surfaces."""

import heapq
import logging
import math
from dataclasses import dataclass, field
from typing import Any

from payton.math.functions import (
    distance,
    dot_product,
    plane_normal,
    sub_vector,
)
from payton.math.vector import Vector3D
from payton.scene.geometry.base import Line, Object
from payton.scene.geometry.mesh import Mesh
from payton.scene.material import GREEN, WIREFRAME, YELLOW, Material

logger = logging.getLogger(__name__)

_UP_AXIS = 2


@dataclass
class NavTriangle:
    index: int
    indices: tuple[int, int, int]
    vertices: tuple[Vector3D, Vector3D, Vector3D]
    centroid: Vector3D
    normal: Vector3D
    bbox_min: Vector3D = field(default_factory=lambda: [0.0, 0.0, 0.0])
    bbox_max: Vector3D = field(default_factory=lambda: [0.0, 0.0, 0.0])
    neighbors: list[int] = field(default_factory=list)
    area_cost: float = 1.0
    flags: int = 0

    def __hash__(self) -> int:
        return self.index

    def _calc_bbox(self) -> None:
        v0, v1, v2 = self.vertices
        self.bbox_min = [
            min(v0[0], v1[0], v2[0]),
            min(v0[1], v1[1], v2[1]),
            min(v0[2], v1[2], v2[2]),
        ]
        self.bbox_max = [
            max(v0[0], v1[0], v2[0]),
            max(v0[1], v1[1], v2[1]),
            max(v0[2], v1[2], v2[2]),
        ]


class NavMeshGraph:
    """Pure-data navigation mesh graph.

    No OpenGL or numpy dependency. Uses payton.math.functions for vector math.
    """

    _GRID_CELL = 2.0

    def __init__(self, max_slope: float = 45.0, max_step: float = 1.0) -> None:
        self.max_slope = max_slope
        self.max_step = max_step
        self.triangles: list[NavTriangle] = []
        self._slope_threshold: float = math.cos(math.radians(max_slope))
        self._up = _UP_AXIS
        self._abs_vertices: list[Vector3D] = []
        self._grid: dict[tuple[int, int], list[int]] = {}

    def build(self, mesh: Mesh) -> None:
        """Build the navmesh graph from a Mesh object.

        Extracts world-space triangles, filters by slope, and builds
        shared-edge adjacency.

        Keyword arguments:
        mesh -- Source Mesh to extract walkable geometry from.
        """
        self.triangles = []
        self._grid = {}
        self._abs_vertices = mesh.absolute_vertices()
        indices = mesh._indices

        edge_map: dict[tuple[int, int], list[int]] = {}

        for face in indices:
            i0, i1, i2 = face[0], face[1], face[2]
            v0: Vector3D = list(self._abs_vertices[i0])
            v1: Vector3D = list(self._abs_vertices[i1])
            v2: Vector3D = list(self._abs_vertices[i2])

            normal = plane_normal(v0, v1, v2)

            if self._slope_threshold > 0 and normal[self._up] < self._slope_threshold:
                continue

            centroid: Vector3D = [
                (v0[0] + v1[0] + v2[0]) / 3.0,
                (v0[1] + v1[1] + v2[1]) / 3.0,
                (v0[2] + v1[2] + v2[2]) / 3.0,
            ]

            tri_index = len(self.triangles)
            tri = NavTriangle(
                index=tri_index,
                indices=(i0, i1, i2),
                vertices=(v0, v1, v2),
                centroid=centroid,
                normal=normal,
            )
            tri._calc_bbox()
            self.triangles.append(tri)

            for edge in [(i0, i1), (i1, i2), (i2, i0)]:
                key = (min(edge[0], edge[1]), max(edge[0], edge[1]))
                edge_map.setdefault(key, []).append(tri_index)

        self._build_adjacency(edge_map)
        self._build_grid()
        logger.info(f"NavMesh built with {len(self.triangles)} walkable triangles")

    def _build_grid(self) -> None:
        self._grid = {}
        for tri in self.triangles:
            cx = tri.centroid[0]
            cy = tri.centroid[1]
            cell = (int(cx / self._GRID_CELL), int(cy / self._GRID_CELL))
            self._grid.setdefault(cell, []).append(tri.index)

    def _build_adjacency(self, edge_map: dict[tuple[int, int], list[int]]) -> None:
        for visitors in edge_map.values():
            if len(visitors) != 2:
                continue
            a, b = visitors
            ta = self.triangles[a]
            tb = self.triangles[b]
            if self.max_step > 0:
                height_diff = abs(ta.centroid[self._up] - tb.centroid[self._up])
                if height_diff > self.max_step:
                    continue
            ta.neighbors.append(b)
            tb.neighbors.append(a)

    def _point_in_bbox(self, x: float, y: float, tri: NavTriangle) -> bool:
        bmin = tri.bbox_min
        bmax = tri.bbox_max
        return bmin[0] <= x <= bmax[0] and bmin[1] <= y <= bmax[1]

    def _point_in_triangle(self, p: Vector3D, v0: Vector3D, v1: Vector3D, v2: Vector3D) -> bool:
        e0 = sub_vector(v2, v0)
        e1 = sub_vector(v1, v0)
        e2 = sub_vector(p, v0)

        d00 = dot_product(e0, e0)
        d01 = dot_product(e0, e1)
        d02 = dot_product(e0, e2)
        d11 = dot_product(e1, e1)
        d12 = dot_product(e1, e2)

        denom = (d00 * d11) - (d01 * d01)
        if abs(denom) < 1e-12:
            return False

        inv_denom = 1.0 / denom
        u = (d11 * d02 - d01 * d12) * inv_denom
        v = (d00 * d12 - d01 * d02) * inv_denom

        return u >= -1e-8 and v >= -1e-8 and (u + v) <= 1.0 + 1e-8

    def _candidate_cells(self, x: float, y: float) -> list[int]:
        cx = int(x / self._GRID_CELL)
        cy = int(y / self._GRID_CELL)
        candidates: list[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                cell = (cx + dx, cy + dy)
                if cell in self._grid:
                    candidates.extend(self._grid[cell])
        return candidates

    def _project_to_triangle(
        self, point: Vector3D, tri: NavTriangle
    ) -> Vector3D | None:
        n = tri.normal
        up = self._up

        if abs(n[up]) < 1e-6:
            return None

        v0 = tri.vertices[0]
        o_minus_v0 = sub_vector(point, v0)
        t = dot_product(n, o_minus_v0) / n[up]

        hit: list[float] = list(point)
        hit[up] -= t

        if self._point_in_triangle(hit, *tri.vertices):
            return hit
        return None

    def _find_containing_triangle(self, point: Vector3D) -> int | None:
        best_index: int | None = None
        best_up: float = float("-inf")
        up = self._up
        px = point[0]
        py = point[1]

        candidates = self._candidate_cells(px, py)
        seen: set[int] = set()

        for tri_idx in candidates:
            if tri_idx in seen:
                continue
            seen.add(tri_idx)
            tri = self.triangles[tri_idx]
            if not self._point_in_bbox(px, py, tri):
                continue
            hit = self._project_to_triangle(point, tri)
            if hit is None:
                continue
            if hit[up] > best_up:
                best_up = hit[up]
                best_index = tri_idx

        return best_index

    def _nearest_triangle(self, point: Vector3D) -> int:
        best_dist = float("inf")
        best_index = 0
        candidates = self._candidate_cells(point[0], point[1])
        seen: set[int] = set()
        for tri_idx in candidates:
            if tri_idx in seen:
                continue
            seen.add(tri_idx)
            d = distance(point, self.triangles[tri_idx].centroid)
            if d < best_dist:
                best_dist = d
                best_index = tri_idx
        return best_index

    def _segment_is_walkable(
        self, a: Vector3D, b: Vector3D, corridor_set: set[int]
    ) -> bool:
        dist = distance(a, b)
        steps = max(2, int(dist / 0.5))
        for s in range(steps + 1):
            frac = s / steps
            sample: Vector3D = [
                a[0] + (b[0] - a[0]) * frac,
                a[1] + (b[1] - a[1]) * frac,
                a[2] + (b[2] - a[2]) * frac,
            ]
            tri = self._find_containing_triangle(sample)
            if tri is None or tri not in corridor_set:
                return False
        return True

    def _smooth_corridor(
        self,
        corridor: list[int],
        start: Vector3D,
        end: Vector3D,
    ) -> list[Vector3D]:
        if len(corridor) <= 1:
            return [list(start), list(end)]

        corridor_set = set(corridor)

        centroids: list[Vector3D] = [list(start)]
        centroids += [self.triangles[idx].centroid for idx in corridor]
        centroids.append(list(end))

        n = len(centroids)
        smoothed: list[Vector3D] = [centroids[0]]
        i = 0
        while i < n - 1:
            furthest = i + 1
            for j in range(n - 1, i, -1):
                if self._segment_is_walkable(centroids[i], centroids[j], corridor_set):
                    furthest = j
                    break
            smoothed.append(centroids[furthest])
            i = furthest

        return smoothed

    def _extract_corridor(
        self, came_from: dict[int, int | None], current: int
    ) -> list[int]:
        corridor: list[int] = []
        node: int | None = current
        while node is not None:
            corridor.append(node)
            node = came_from.get(node)
        corridor.reverse()
        return corridor

    def find_path(self, start: Vector3D, end: Vector3D) -> list[Vector3D]:
        """Find a smoothed path from start to end point on the navmesh.

        Returns an ordered list of waypoints (Vector3D positions).
        Returns an empty list if no path exists.
        The path is smoothed using a corridor-aware shortcut algorithm.

        Keyword arguments:
        start -- Starting world position.
        end -- Goal world position.
        """
        if not self.triangles:
            return []

        start_idx = self._find_containing_triangle(start)
        if start_idx is None:
            start_idx = self._nearest_triangle(start)
        end_idx = self._find_containing_triangle(end)
        if end_idx is None:
            end_idx = self._nearest_triangle(end)

        projected_start = self.nearest_point(start)
        projected_end = self.nearest_point(end)

        if start_idx == end_idx:
            return [projected_start, projected_end]

        counter = 0
        open_set: list[tuple[float, int, int]] = []
        g_score: dict[int, float] = {start_idx: 0.0}
        came_from: dict[int, int | None] = {start_idx: None}

        end_centroid = self.triangles[end_idx].centroid
        start_centroid = self.triangles[start_idx].centroid
        h_start = distance(start_centroid, end_centroid)
        heapq.heappush(open_set, (h_start, counter, start_idx))
        counter += 1

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == end_idx:
                corridor = self._extract_corridor(came_from, current)
                return self._smooth_corridor(corridor, start, end)

            for neighbor in self.triangles[current].neighbors:
                tri_cur = self.triangles[current]
                tri_nxt = self.triangles[neighbor]
                move_cost = distance(tri_cur.centroid, tri_nxt.centroid) * max(
                    tri_cur.area_cost, tri_nxt.area_cost
                )
                tentative_g = g_score.get(current, float("inf")) + move_cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    h = distance(tri_nxt.centroid, end_centroid)
                    f_score = tentative_g + h
                    heapq.heappush(open_set, (f_score, counter, neighbor))
                    counter += 1

        return []

    def nearest_point(self, point: Vector3D) -> Vector3D:
        """Project a point onto the nearest walkable surface.

        Keyword arguments:
        point -- World position to project.
        """
        if not self.triangles:
            return point

        tri_idx = self._find_containing_triangle(point)
        if tri_idx is not None:
            tri = self.triangles[tri_idx]
            hit = self._project_to_triangle(point, tri)
            if hit is not None:
                return hit

        nearest = self.triangles[self._nearest_triangle(point)]
        return nearest.centroid

    def is_walkable(self, point: Vector3D) -> bool:
        """Check if a point projects vertically onto a walkable triangle.

        The point is projected down along the Z axis and tested against
        all navmesh triangles. Returns True if any walkable triangle
        lies directly beneath the given point.

        Keyword arguments:
        point -- World position to test.
        """
        if not self.triangles:
            return False
        return self._find_containing_triangle(point) is not None


class NavMesh(Object):
    """Navigation mesh object with debug visualization.

    Extends Object so it integrates with the Scene render loop.
    The debug wireframe overlay shows walkable triangles in green.

    Usage::

        navmesh = NavMesh()
        navmesh.build_from_mesh(terrain_mesh)
        scene.add_object("navmesh", navmesh)

        path = navmesh.graph.find_path(start, end)
    """

    def __init__(self, max_slope: float = 45.0, max_step: float = 1.0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.graph = NavMeshGraph(max_slope=max_slope, max_step=max_step)
        self._path_line: Object | None = None
        self._walkable_material = Material(color=GREEN)
        self._walkable_material.display = WIREFRAME
        self._walkable_material.lights = False

    def build_from_mesh(self, mesh: Mesh) -> None:
        """Build the navmesh graph from a Mesh and create debug visualization.

        Keyword arguments:
        mesh -- Source Mesh to extract walkable geometry from.
        """
        self.graph.build(mesh)
        self._build_visualization()

    def _build_visualization(self) -> None:
        self._vertices = []
        self._indices = []
        self._normals = []
        self._texcoords = []

        for tri in self.graph.triangles:
            i = len(self._vertices)
            v0, v1, v2 = tri.vertices
            self._vertices.extend([list(v0), list(v1), list(v2)])
            self._indices.append([i, i + 1, i + 2])
            for _ in range(3):
                self._normals.append(tri.normal)
                self._texcoords.append([0.0, 0.0])

        self._vertex_count = len(self._vertices)
        self.material = self._walkable_material
        self.refresh()

    def highlight_path(self, path: list[Vector3D]) -> None:
        """Display a yellow line for the active path.

        Keyword arguments:
        path -- Ordered list of waypoints from find_path().
        """
        self.clear_path()
        if not path:
            return

        line = Line(vertices=path, color=YELLOW)
        self._path_line = line
        self.add_child("_navmesh_path", line)

    def clear_path(self) -> None:
        if "_navmesh_path" in self.children:
            self.children["_navmesh_path"].destroy()
            del self.children["_navmesh_path"]
            self._path_line = None
