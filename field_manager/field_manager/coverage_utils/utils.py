#!/usr/bin/env python

from typing import Any, List, Sequence, TypeVar, Union

import matplotlib.colors as mcolors
import numpy as np
from numpy.typing import NDArray
from std_msgs.msg import ColorRGBA, MultiArrayDimension

color_list = ["r", "g", "b", "m", "c", "y"]


def padding(
    original_array: Union[Sequence, NDArray],
    return_list_length: int = 3,
    padding_value: float = 0.0,
) -> List[float]:
    """To handle 1~3 dimensions, fill the latter part with 0 or a specified value

    Args:
        original_array (Sequence): The original array
        return_list_length (int): The size of the list after padding. Defaults to 3.
        padding_value (float): The value to use for padding. Defaults to 0.

    Returns:
        List[float]: The padded result list

    Note:
        We set the default value of return_list_length to 3, assuming that the result will be used as a Point or Vector3.
    """
    original_array_length = len(original_array)
    return [(original_array[i] if i < original_array_length else padding_value) for i in range(return_list_length)]


def get_color_rgba(color: str, alpha: float = 1.0) -> ColorRGBA:
    r, g, b, a = mcolors.to_rgba(color, alpha)
    return ColorRGBA(r=r, g=g, b=b, a=a)


MultiArray = TypeVar("MultiArray")


def ndarray_to_multiarray(multiarray_type: MultiArray, ndarray: NDArray) -> MultiArray:
    """Convert numpy.ndarray to multiarray"""
    multiarray = multiarray_type()  # type: ignore
    multiarray.layout.dim = [
        MultiArrayDimension(label=f"dim{i}", size=ndarray.shape[i], stride=ndarray.shape[i] * ndarray.dtype.itemsize)
        for i in range(ndarray.ndim)
    ]
    multiarray.data = ndarray.reshape(1, -1)[0].tolist()  # type: ignore
    return multiarray  # type: ignore


def multiarray_to_ndarray(pytype: Any, dtype: Any, multiarray: MultiArray) -> NDArray:
    """Convert multiarray to numpy.ndarray"""
    dims = [multiarray.layout.dim[i].size for i in range(len(multiarray.layout.dim))]  # type: ignore
    return np.array(multiarray.data, dtype=pytype).reshape(dims).astype(dtype)  # type: ignore
