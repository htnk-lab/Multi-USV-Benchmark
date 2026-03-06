#!/usr/bin/env python

import matplotlib.colors as mcolors
from std_msgs.msg import ColorRGBA

# Define the colors used for various visualizations
color_list = ["r", "g", "b", "m", "c", "y"]


def get_color_rgba(color: str, alpha: float = 1.0) -> ColorRGBA:
    r, g, b, a = mcolors.to_rgba(color, alpha)
    return ColorRGBA(r=r, g=g, b=b, a=a)
