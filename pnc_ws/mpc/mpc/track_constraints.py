#%%
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from casadi import MX, DM, Function, horzcat
# PYTHONPATH shenanigans to import custom_opt
from sys import path
from os.path import join, dirname, abspath

#%%
def loopify(path):
    assert path.shape[1]==2, f"loopify expected path of shape (n, 2) but got {path.shape}"
    return np.concatenate((path[-1:], path), axis=0).T

def get_interpolants(path, interp_method):
    if path.shape[0]!=2 and path.shape[1]==2: path = path.T 
    if np.linalg.norm(path[:, 0]-path[:, -1])>0.001: path = loopify(path.T)
    # create tgrid which matches the points
    # has nothing to do with actual time, just parameterizes
    # the path with # Math: t \in [0, 1]
    # we use `endpoint=True` because we loopified the path. 
    # This way, t=0 and t=1 correspond to the same point.

    tgrid = np.linspace(0, 1, path.shape[1], endpoint=True, dtype=float).tolist()

    # init symbolic variables
    p = MX.sym('p', 2)
    t = MX.sym('t')

    # interpolate points to get parametric path
    xlut = ca.interpolant('xlut', interp_method, [tgrid], path[0].tolist())
    ylut = ca.interpolant('ylut', interp_method, [tgrid], path[1].tolist())
    # get derivatives of parametric path
    xjac = Function('xjac', [t], [ca.jacobian(xlut(t), t)])
    yjac = Function('yjac', [t], [ca.jacobian(ylut(t), t)])
    return xlut, ylut, xjac, yjac

def get_g_one_side(path, **opts):
    """gets a function `g(x): R²->R` approximating the turning number of `path` about `x`

    Args:
        path (numpy.ndarray): array of points to convert to path. must be 2-index, and one index must be length 2.
        interp_method (str, optional): the interpolation method to use. see docs for `casadi.interpolant`. Defaults to `'linear'`.
        n_points (int, optional): number of points along the path to sample for the approximation. Defaults to the number of points in `path`.
    Returns:
        casadi.Function: The function `g(x)`
    """
    # transpose if needed
    if path.shape[0]!=2 and path.shape[1]==2: path = path.T 

    # if neccessary, add a final point which equals the initial point, closing the loop
    if np.linalg.norm(path[:, 0]-path[:, -1])>0.001: path = loopify(path.T).T

    # update options from kwargs
    interp_method = opts['interp_method'] if 'interp_method' in opts else 'linear'
    n_points = opts['n_points'] if 'n_points' in opts else path.shape[1]

    xlut, ylut, xjac, yjac = get_interpolants(path, interp_method)

    # new tgrid with correct number of points according to options
    tgrid = np.linspace(0, 1, n_points, endpoint=False, dtype=float).tolist()

    # okay so here's the actual math
    # the formula is as follows:
    # Math: \int_{0}^{2\pi}\frac{\dot{x}(y-y_p) - \dot{y}(x-x_p)}{(x-x_p)^2+(y-y_p)^2}\,dt
    # Math: \text{where }\begin{bmatrix}x(t)\\y(t)\end{bmatrix} \text{ parameterizes our path, and }p=\begin{bmatrix}x_p\\y_p\end{bmatrix} \text{is the point we're considering.}
    # This comes from the "turning number" or "winding number" of the curve about the point p,
    # or in other words, how many total times the path winds counterclockwise around our point.
    # In the continuous case, this integral expression is discontinuous at the curve,
    # but we're discretizing anyway, so it ends up close to that but still differentiable, which is very nice.
    
    p = MX.sym('p', 2)
    
    dt = 1/n_points # Dt = 1 so dt=1/n_points
    acc = 0
    for t in tgrid:
        num = xjac(t)*(ylut(t)-p[1]) - yjac(t)*(xlut(t)-p[0])
        denom = ((xlut(t)-p[0])**2 + (ylut(t)-p[1])**2)

        acc += (num/denom)*dt

    return Function('g', [p], [acc])

def get_full_g(left, right, **opts): 
    # fix transpose, if it's wrong. so user doesn't have to keep track of this.
    if left.shape[0]  != 2 and left.shape[1]  == 2: left  = left.T
    if right.shape[0] != 2 and right.shape[1] == 2: right = right.T

    p = MX.sym('p', 2)
    gl = get_g_one_side(left, **opts)
    gr = get_g_one_side(right, **opts)

    center = DM((sum(left.T)/len(left.T)).flatten().tolist())
    size = sum([np.linalg.norm(i) for i in left.T])

    return Function('g', [p], [ca.tanh(-gl(p)+gr(p)+3)])#(0.1*ca.sumsqr(p-center)/size+10)]) # serial, unroll, openmp, thread, maxthreads (int)
# %%
