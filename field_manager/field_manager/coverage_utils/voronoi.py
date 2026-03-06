#!/usr/bin/env python

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from numpy.typing import NDArray


@dataclass
class Voronoi:
    """Manage Voronoi regions

    Attributes:
        p (float): p-norm
        radius (float): radius to calculate r-limited voronoi.
                        Used for FOV settings. Defaults to float("inf")

    Note:
        Assign True to positions corresponding to discrete points within each region, and False to all other positions.
        The notation follows H.Dan et al. 2020.

        voronoi_region (NDArray): Voronoi region (V_{i})
        fov_region (NDArray): Field of view (B_{i}) based on circular sensor model.
                              By default, the radius is set to infinity to consider only the Voronoi region.
        sensing_region (NDArray): Sensing region (S_{i}=V_{i} \bigcap B_{i})
    """

    p: float = 2
    radius: float = float("inf")

    def calc_tesselation(
        self,
        agent_position: NDArray,
        neighbor_agent_position_list: List[NDArray],
        phi: NDArray,
        grid_map: List[NDArray],
        point_density: float,
    ) -> Tuple[NDArray, List[NDArray], NDArray]:
        """Calculate Voronoi tessellation

        Args:
            agent_position (NDArray): Voronoi region generator point
            neighbor_agent_position_list (List[NDArray]): Neighbor generator points.
            phi (NDArray): Importance map
            grid_map (List[NDArray]): Positions of discrete points. [x_grid_map, y_grid_map, ...]
            point_density (float): Discrete point density.

        Returns:
            Tuple[NDArray, List[NDArray], NDArray]:
            Voronoi centroid, sensing region grid points, and sensing region mask.
        """
        dim = len(grid_map)

        voronoi_region: NDArray = np.ones_like(grid_map[0], dtype=np.bool_)

        assert self.p >= 1 and self.radius > 0
        distance_from_agent = sum([abs(grid_map[i] - agent_position[i]) ** self.p for i in range(dim)]) ** (1 / self.p)
        fov_region = distance_from_agent < self.radius

        for neighbor_agent_position in neighbor_agent_position_list:
            distance_from_neighbor = sum(
                [abs(grid_map[i] - neighbor_agent_position[i]) ** self.p for i in range(dim)]
            ) ** (1 / self.p)

            near_region = distance_from_neighbor > distance_from_agent
            voronoi_region *= near_region

        sensing_region = voronoi_region * fov_region

        centroid_position = self.calc_centroid_position(grid_map, phi, sensing_region, point_density, dim)
        sensing_region_grid_points = [grid_map[i][sensing_region] for i in range(dim)]
        return centroid_position, sensing_region_grid_points, sensing_region

    @staticmethod
    def calc_centroid_position(
        grid_map: List[NDArray], phi: NDArray, region: NDArray, point_density: float, dim: int
    ) -> NDArray:
        weighted_grid_map = grid_map * phi * region
        mass = np.sum(phi * region) * point_density
        assert mass > 0

        centroid_position: NDArray = np.array([weighted_grid_map[i].sum() for i in range(dim)]) * point_density / mass
        return centroid_position
