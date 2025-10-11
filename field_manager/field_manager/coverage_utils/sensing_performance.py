#!/usr/bin/env python

from typing import Union

from sympy import Matrix, Symbol, exp, lambdify, rot_ccw_axis3, symbols


class SensingPerformance:
    """
    Note:
        The symbolic variables used to define the performance function are assumed to be real numbers.

        p_i: agent position \in \mathbb{R}^2
        q_j: field point \in \mathbb{R}^2
        sigma: tuning parameter
    """

    # class variables
    p_ix, p_iy = symbols(r"p_ix, p_iy", real=True)  # agent position
    p_i = Matrix([p_ix, p_iy])  # agent position
    q_j = Matrix(symbols(r"q_jx,q_jy", real=True))  # field point

    # sensing performance function using the euclidean distance from agent position to field point
    sigma = Symbol(r"\sigma", real=True)
    f = exp(-(p_i - q_j).dot(p_i - q_j) / (2 * sigma**2))
    f_ufunc = lambdify([p_i, q_j, sigma], f)
