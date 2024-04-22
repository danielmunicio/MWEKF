#%%
import numpy as np
from numpy.linalg import norm
# import triangle as tr
import casadi as ca
import matplotlib.pyplot as plt

# side1 = [
#     [2,9.6,14.3,17.5,18,17.6,22.3,28.7,34.2,40.3,42.85,42.7,30.8,22.3,-2.6,-9,-17.7,-23,-24.4,-22.7,-18.2,-13.7,-7.9,-1.4,39.8,15.8,6.3, 35.],
#     [-13.4,-12.2,-8,-3.05,3.5,9.6,-6.4,-8,-6.75,0.6,10.55,20.9,36.5,36.2,32.16,38.5,39.7,35.,28.45,20.55,11.2,3,-3.45,-10.5,30.36,33.6,32.3, 33.5,]
# ]
# side2 = [
#     [1.6,6.7,10.86,11.1,11.74,17.9,23.7,24.02,28.7,34.5,36.3,34.9,31.8,26.2,19.8,12.7,4.16,-4.5,-10.05,-14.8,-17.6,-16.2,-12.6,-10.1,-2.3,10,-6.4,24.1],
#     [-6.4,-6.8,2,7.5,14.4,17.4,13.7,0.8,-0.4,3.26,12.6,20.3,26.8,30.,28.5,27.2,26.1,25.84,31.1,32.2,27.9,20.4,13.3,7.1,-2.,-3.7,3.34,8.75]
# ]

# arr1 = np.array(side1)*1
# arr2 = np.array(side2)*1
# set1 = set(zip(*arr1))
# set2 = set(zip(*arr2))


class PairPath:
    def __init__(self, l, r, n=100):
        self.n = n
        self.l = self._ensure_shape(l)
        self.r = self._ensure_shape(r)
        self.l = self._remove_duplicates(self.l)
        self.r = self._remove_duplicates(self.r)
        self.c, self.pairs = self._get_center(self.l, self.r)
        self.c, self.indices = self._fix_order(self.c)
        self.l = self.l[self.pairs[0]][self.indices]
        self.r = self.r[self.pairs[1]][self.indices]
        
        # self.l = self._remove_duplicates(self.l)
        # self.r = self._remove_duplicates(self.r)

        # self.l = self._interp_add_pts(self.l, self.n)
        # self.r = self._interp_add_pts(self.r, self.n)

        self.l = self._interp_add_pts(self.l, 2*self.n)
        self.r = self._interp_add_pts(self.r, 2*self.n)
        self.c = self._interp_add_pts(self.c, self.n)
        
        l = []
        r = []
        for i in self.c:
            l.append(self.l[np.argmin(norm(self.l-i, axis=1))])
            r.append(self.r[np.argmin(norm(self.r-i, axis=1))])

        self.l = np.array(l)
        self.r = np.array(r)


    
    @property
    def left(self): return self.l
    @property
    def right(self): return self.r
    def __len__(self): return self.n
    def _remove_duplicates(self, path, thresh=0.2):
        path = path.copy()
        i = 0
        while i<len(path):
            norms = norm(path-i, axis=1)
            path = np.delete(path, norms<thresh, axis=0)
            i += 1
        return path
    def _interp_add_pts(self, path, n=200, arc_len=True):
        path = np.concatenate([path[-1:], path], axis=0) # loopify
        if arc_len:
            tgrid = np.array([0.]+np.cumsum(norm(np.diff(path, axis=0), axis=1)+0.001).tolist())
            tgrid = tgrid/tgrid[-1]
        else:
            tgrid = np.linspace(0, 1, path.shape[0], endpoint=True, dtype=float).tolist()
        # interpolate points to get parametric path
        xlut = ca.interpolant('xlut', 'bspline', [tgrid], path[:, 0].tolist())
        ylut = ca.interpolant('ylut', 'bspline', [tgrid], path[:, 1].tolist())
        tgrid = np.linspace(0, 1, n, endpoint=False, dtype=float).tolist()

        return np.vstack([
            np.array(xlut(tgrid), dtype=float).flatten(), 
            np.array(ylut(tgrid), dtype=float).flatten()
        ]).T
    
    def _ensure_shape(self, path):
        try: new_path = np.squeeze(list(path))
        except TypeError as e: raise TypeError(f"wrong type for path: {e}") # e is something like "'int' object is not iterable"
        
        assert new_path.ndim==2, f"exactly two dimensions must have size >1. got invalid shape {path.shape}"
        assert 2 in new_path.shape, f"one dimension of path must have size 2. got invalid shape {path.shape}"
        if new_path.shape[1]==2:
            return new_path
        else:
            return new_path.T
    def _get_center(self, left, right):
        pairs = set()
        for i in range(left.shape[0]):
            pairs.add( (i, np.argmin(norm(right-left[i], axis=1))) )
        for i in range(right.shape[0]):
            pairs.add( (np.argmin(norm(left-right[i], axis=1)), i) )
        pairs = np.array(list(pairs)).T
        center = (left[pairs[0]]+right[pairs[1]])/2
        return center, pairs
        # plt.scatter(*left.T, color='tab:blue')
        # plt.scatter(*right.T, color='tab:orange')
        # plt.scatter(*center.T, color='black')
        # plt.show()

    def _fix_order(self, path):
        # `indices` will store the indices of path in the proper order
        indices = [0]
        path_old = path
        path = path.copy() # make sure we don't modify the original `path` object
        path[0]=np.inf
        for _ in range(path.shape[0]):
            # find closest point
            dists = np.linalg.norm(path-path_old[indices[-1]], axis=1)
            idx = np.argmin(dists)
            indices.append(idx)
            # mark node as visited so we don't go through it again
            path[indices] = np.inf
        indices = np.array(indices)
        return path_old[indices], indices
    
    def plot(self, ax):
        ax.plot(*np.vstack([self.l, self.l[:1]]).T, color='tab:blue',   label='left')
        ax.plot(*np.vstack([self.r, self.r[:1]]).T, color='tab:orange', label='right')
        # ax.scatter(*self.c.T, color='black',      label='center')
        ax.legend()
        # plt.show()
    
    def get_internal_point(self):
        return 

# p = Path(set1, set2)
# p.plot(plt.axes())
#%%
if __name__ == '__main__':
    from global_opt_compiled import CompiledGlobalOpt, CompiledGlobalOptLoader
    from time import perf_counter

    compile_new_solver = False

    if compile_new_solver:
        g = CompiledGlobalOpt(100)
        g.construct_solver(generate_c=True, compile_c=True)
    g = CompiledGlobalOptLoader(100)
    tic = perf_counter()
    p = PairPath(set1, set2)
    res = g.solve(p.right, p.left)
    toc = perf_counter()
    print(f'total solve time: {toc-tic:.2f}s')


    fig, ax = plt.subplots(1, 1, sharex=True, sharey=True)

    ax.scatter(*arr1, color='black', label='path input points')
    ax.scatter(*arr2, color='black')
    # ax.plot(arr1, color='black')
    # ax.plot(arr2, color='black')
    p.plot(ax)
    # g.traj.path.plot(ax[1])


    s1 = ax.scatter(*g.soln['xy'].T, c=res['z'][1:, 3], cmap='coolwarm')
    fig.colorbar(s1, ax=ax).set_label('Speed ($[m][s]^{-1}$)')
    # for i in range(p.l.shape[0]):
    #     ax.plot(*np.vstack([p.l[i], p.r[i]]).T)
    plt.show()
    
# %%
