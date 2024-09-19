#%%
import casadi as ca
import numpy as np
from casadi import MX, DM
from .track_constraints import get_interpolants
from sys import path
import matplotlib.pyplot as plt

import triangle as tr
import multiprocessing as mp
import itertools
import os

def get_g_triangulation(left, right, interior_point, n_pts=200, n_meshes=2, sink=0.1, tr_opts='D', return_mats=False):
    assert n_meshes >= 1, f"must have at least one mesh! you passed n_meshes={n_meshes}."

    xlutl, ylutl, _, _ = get_interpolants(left, 'bspline')
    xlutr, ylutr, _, _ = get_interpolants(right, 'bspline')

    n_pts = int(n_pts/n_meshes) # since we're splitting into multiple triangulations
    tgrid = np.linspace(0, 1, n_pts, endpoint=False, dtype=float)

    dt = 1/n_pts # distance in t (units of tgrid, 0=start, 1=end) to offset each mesh's points

    l = [
        np.hstack([xlutl(tgrid + i/n_meshes * dt), ylutl(tgrid + i/n_meshes * dt)]).astype(float)
        for i in range(n_meshes)
    ]
    r = [
        np.hstack([xlutr(tgrid + i/n_meshes * dt), ylutr(tgrid + i/n_meshes * dt)]).astype(float)
        for i in range(n_meshes)
    ]
    
    polygon = lambda path: {
        'vertices': path,
        'segments': np.array([[path.shape[0]-1, 0]]+[[i, i+1] for i in range(path.shape[0]-1)], dtype=int)
    }

    pls = [polygon(i) for i in l]
    prs = [polygon(i) for i in r]

    p_i = lambda i: {
        'vertices': pls[i]['vertices'].tolist() + prs[i]['vertices'].tolist(),
        'segments': pls[i]['segments'].tolist() + (prs[i]['segments']+len(pls[i]['segments'])).tolist(),
        'holes':    [interior_point]
    }
    ps = [p_i(i) for i in range(n_meshes)]

    ts = [tr.triangulate(ps[i], 'p'+tr_opts) for i in range(n_meshes)]

    # tr.plot(plt.axes(), **ts[0])
    # plt.show()
    mats_out = []
    g = 0
    p = MX.sym('p', 2)
    p_aug = ca.vertcat(p, DM(1))
    I = np.eye(3)
    for t in ts:
        verts = t['vertices']
        trigs = t['triangles']
        mats = DM(
            np.vstack([
                np.linalg.solve(
                    np.vstack([
                        verts[list(i)].T, 
                        np.ones((1, 3))
                    ]),
                    I
                ) for i in trigs
            ])
        )
        mats_out.append(mats)
        s = mats@p_aug
        res = ca.reshape((ca.sign(s)-1)*s, (3, t['triangles'].shape[0]))

        x = MX.sym('x', 3)
        # s = (ca.sign(x)-1)*x
        # f = ca.Function('f', [x], [ca.sum1(res)])

        g+= ca.mmin(ca.sum1(res))


    opts = {'post_expand': True} # made it abt 47% faster on my computer
    g = ca.Function('g', [p], [g-sink], opts)
    
    if return_mats: return mats_out, g
    return g


def find_g_fast(mats, p_aug):
    x = mats@p_aug
    return np.max(np.sum((np.signbit(x)*x).reshape((int(mats.shape[0]/3), 3, x.shape[-1])), axis=1), axis=0)

def get_interp_g_fast(mats, xrange, yrange, resolution=0.1, n_cores=14):
    mats = mats.astype(np.float32)

    x = np.arange(min(xrange), max(xrange), resolution, np.float32)
    y = np.arange(min(yrange), max(yrange), resolution, np.float32)

    pts = np.meshgrid(x, y)
    pts = np.vstack([pts[0].flatten(), pts[1].flatten(), np.ones(pts[0].flatten().shape[0], dtype=np.float32)])

    z = np.hstack(mp.Pool(n_cores).starmap(find_g_fast, zip(np.repeat(mats[np.newaxis], n_cores, axis=0), np.array_split(pts, n_cores, 1))))
    
    return ca.interpolant('interp_g', 'linear', [x, y], z)
    # z = np.hstack(*mp.Pool(n_cores).starmap(find_g_fast, [itertools.repeat(mats), np.array_split(pts, n_cores, 1)]))

def get_interp_g(g, xrange, yrange, resolution=0.1, n_cores=14, compile_g=True, compile_opt_flag='-Ofast'):
    x = np.arange(min(xrange), max(xrange), resolution)
    y = np.arange(min(yrange), max(yrange), resolution)

    pts = np.meshgrid(x, y)
    pts = np.vstack([pts[0].flatten(), pts[1].flatten()]).T

    if compile_g:
        g.generate('g_gen.c')
        os.system(f'gcc -fPIC {compile_opt_flag} -shared g_gen.c -o g_gen.so')
        g = ca.external('g', './g_gen.so')

    # multiprocessing! cut runtime down by n_cores times
    z = np.array(ca.horzcat(
        *mp.Pool(n_cores).map(g, np.array_split(pts.T, n_cores, 1))
    ))[0]
    
    return ca.interpolant('interp_g', 'linear', [x, y], z)


#%%
if __name__ == '__main__':
    import os
    from time import perf_counter
    jit = False
    path = traj.Path(spacing=5)
    tic = perf_counter()
    mats, g = get_g_triangulation(path.l, path.r, [0, 20], n_pts=100, n_meshes=1, return_mats=True)
    # g.generate('g_gen.c')
    # # get_interp_g_fast(np.array(mats[0]), (-100, 100), (-100, 100), 1)
    # #%%
    # if jit:
    #     C = ca.Importer('g_gen.c', 'clang')
    #     g = ca.external('g', C)
    # else:
    #     tic2 = perf_counter()
    #     os.system('gcc -fPIC -Ofast -shared g_gen.c -o g_gen.so')
    #     print(f'compile time: {perf_counter()-tic2:.3f}s')
    #     g = ca.external('g', './g_gen.so')
    print(f'total pre-execution time: {perf_counter()-tic:.3f}s')
    g = get_interp_g(g, (-120, 120), (-120, 120), 0.2)
    print('time: ', perf_counter()-tic)
    # %%
    n=160
    div=1
    x, y = np.meshgrid(np.arange(-n, n, div), 
                    np.arange(-n, n, div))
    z = np.array(
        ca.reshape(
            g(
                ca.horzcat(
                    DM(x.flatten()), 
                    DM(y.flatten())
                ).T
            ),
            x.shape,
        )
    )
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(x, y, z, cmap='coolwarm')

    path.plot(ax)
    plt.show()
# %%
