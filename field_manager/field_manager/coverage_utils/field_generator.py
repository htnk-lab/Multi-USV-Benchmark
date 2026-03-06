#!/usr/bin/env python

from dataclasses import dataclass, field
from typing import List

import numpy as np
from numpy.typing import NDArray


@dataclass
class FieldGenerator:
    """Field Generator

    Attributes:
        grid_accuracy (NDArray): accuracy of field discretization. The number of lattice points in each axis.
        limit (NDArray): field range for each axis. [[x_min, x_max], ...]
        linspace (List[NDArray]): coordinate set for each axis for field generation.
        grid_span (NDArray): width of the field that each lattice point is responsible.

    Note:
        By setting the indexing argument of np.meshgrid to 'ij', grid_map returns x, y, z in that order
    """

    grid_accuracy: NDArray
    limit: NDArray
    linspace: List[NDArray] = field(init=False)
    grid_span: NDArray = field(init=False)

    def __post_init__(self) -> None:
        dim = len(self.grid_accuracy)

        self.grid_span = (self.limit[0:dim, 1] - self.limit[0:dim, 0]) / self.grid_accuracy
        # offset to fill the field without gaps
        self.linspace = [
            np.linspace(
                start=self.limit[i][0] + self.grid_span[i] / 2,
                stop=self.limit[i][1] - self.grid_span[i] / 2,
                num=self.grid_accuracy[i],
            )
            for i in range(dim)
        ]

    def generate_phi(self) -> NDArray:
        return np.ones(self.grid_accuracy)

    def generate_grid_map(self) -> List[NDArray]:
        """generate grid map

        Returns:
            List[NDArray]: lattice points of each coordinate of the field. [x_grid_map, y_grid_map, ...]
        """
        return np.meshgrid(*self.linspace, indexing="ij")
