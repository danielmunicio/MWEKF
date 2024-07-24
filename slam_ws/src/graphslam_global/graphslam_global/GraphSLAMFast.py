from time import perf_counter

import numpy as np
import scipy as sp

class GraphSLAMFast:
    def __init__(self, x0: np.ndarray = np.array([0., 0.]), maxrows: int = 3000, maxcols: int = 3000, max_landmark_distance: float = 1, dx_weight: float = 2.0, z_weight: float = 1.0):
        """initialize GraphSLAMFast object

        Args:
            x0 (np.ndarray, optional): array [x, y] of initial state. Defaults to np.array([0.0, 0.0]).
            maxrows (int, optional): max number of equations this GraphSLAMFast can store (GraphSLAMFast is not dynamically sized, but it's cheap to make this big). Defaults to 3000.
            maxcols (int, optional): max number of variables this GraphSLAMFast can store (GraphSLAMFast is not dynamically sized, but it's cheap to make this big). Defaults to 3000.
            max_landmark_distance (float, optional): how far away landmarks can be from the closest landmark guess before they are recognized as independent. Defaults to 1.
            dx_weight (float, optional): weight (certainty) for odometry measurements. Defaults to 2.0.
            z_weight (float, optional): weight (certainty) for landmark measurements. Defaults to 1.0.
        """
        self.max_landmark_distance = max_landmark_distance
        self.maxrows=maxrows
        self.maxcols=maxcols
        self.dx_weight = dx_weight
        self.z_weight = z_weight
        self.A = sp.sparse.lil_array((self.maxrows, self.maxcols), dtype=np.float64)
        self.b = np.zeros((self.maxcols), np.float64)
        self.A[0, 0] = self.dx_weight
        self.A[1, 1] = self.dx_weight
        self.b[0] = x0[0]*self.dx_weight
        self.b[1] = x0[1]*self.dx_weight

        self.nvars = 2
        self.neqns = 2

        self.x = [0]
        self.l = []

        self.z  = []
        self.d = [0]

        self.xhat = np.zeros((1, 2))
        self.lhat = np.zeros((0, 2))
        self.color = np.zeros(0, dtype=np.uint8)
        self.xhat[0, :] = x0

    # @profile
    def update_graph(self, dx: np.ndarray, z: np.ndarray, color: np.ndarray):
        """update graph

        Args:
            dx (ndarray): difference in position from last update. [dx, dy]
            z (ndarray): measurements to landmakrs. [[zx1, zy1], [zx2, zy2], ..., [zxn, zyn]]
            _color (ndarray): not implemented yet
        """
        color = color.astype(np.uint8)
        # first add two equations and two variables
        # for the next position and the dx
        self.x.append(self.nvars)
        self.d.append(self.neqns)
        self.nvars += 2
        self.neqns += 2

        self.A[self.d[-1], self.x[-1]] = self.dx_weight
        self.A[self.d[-1]+1, self.x[-1]+1] = self.dx_weight
        self.A[self.d[-1], self.x[-2]] = -self.dx_weight
        self.A[self.d[-1]+1, self.x[-2]+1] = -self.dx_weight
        self.b[self.d[-1]] = dx[0]*self.dx_weight
        self.b[self.d[-1]+1] = dx[1]*self.dx_weight

        # now add the guess for this position to xhat
        self.xhat = np.append(self.xhat, self.xhat[-1:, :] + dx, axis=0)

        # now do data association
        # to find the which landmarks correspond
        # to which measurements

        # dists has one row per measurement and one column for each landmark
        # can probably do this faster if we norm at the same time as broadcasting but can't do that easily
        for c in np.unique(color):
            z_c = z[color==c]
            # selfz_c = self.z[self.color==c]
            dists = np.linalg.norm((self.xhat[-1, :]+z_c)[:, np.newaxis, :] - self.lhat[self.color==c], axis=2)
            if len(self.lhat[self.color==c])==0:
                l_idxs=np.zeros(len(z_c), dtype=int)
                l_dists=np.zeros(len(z_c))+np.Inf
            else:
                l_idxs = np.argmin(dists, axis=1)
                l_dists = np.min(dists, axis=1)
            
            for i in range(len(z_c)):
                # if we haven't seen thihs landmark before, add it
                if l_dists[i] > self.max_landmark_distance:
                    self.l.append(self.nvars)
                    self.nvars += 2
                    self.lhat = np.append(self.lhat, self.xhat[-1, :]+z_c[i][np.newaxis], axis=0)
                    self.color = np.append(self.color, c)
                    l_idxs[i] = np.sum(self.color==c)-1 # len(self.l[self.color==c])-1  but self.l is a list so not bool mask indexable

                l = np.array(self.l)[self.color==c]
                # now we write the equation l-x=z
                self.z.append(self.neqns)
                self.neqns += 2
                self.A[self.z[-1], l[l_idxs[i]]] = self.z_weight
                self.A[self.z[-1]+1, l[l_idxs[i]]+1] = self.z_weight
                self.A[self.z[-1], self.x[-1]] = -self.z_weight
                self.A[self.z[-1]+1, self.x[-1]+1] = -self.z_weight
                self.b[self.z[-1]] = z_c[i, 0]*self.z_weight
                self.b[self.z[-1]+1] = z_c[i, 1]*self.z_weight
    # @profile
    def solve_graph(self):
        """solve graph. does not return results.
        """
        # converting to csr here is much more efficient than using csr to begin with
        # because lil is much more efficient to construct
        # and csr is much better for math
        # and the conversion is pretty fast since they're both row-based
        A = self.A.tocsr()[:self.neqns, :self.nvars]
        b = self.b[:self.neqns]

        # ok options to solve the system
        # 1. sp.sparse.linalg.lsqr(A, b) takes about 4000 us
        # 2. adding x0 given by xhat and lhat reduces that to 3000 us
        # 3. just A.T@A \ A.T@b with sp.sparse.linalg.spsolve is only 820 us
        # Adding `permc_spec="NATURAL"` reduced solve time from 820 to 630 us. not entirely
        # sure what that does; best guess is something about col/row permutations, which we
        # shouldn't really do since our matrix is already fairly nice
        # use_umfpack=False is necessary because for some reason it doesn't work with ufmpack. likely scipy bug

        # soln = sp.sparse.linalg.lsqr(A, b)[0]

        # x0 = np.zeros(self.nvars)
        # for idx, val in zip(self.x+self.l, np.vstack((self.xhat, self.lhat))):
        #     x0[idx:idx+2] = val

        # soln = sp.sparse.linalg.lsqr(A, b, x0=x0)[0]

        soln = sp.sparse.linalg.spsolve(A.T@A, A.T@b, permc_spec="NATURAL", use_umfpack=False)

        
        for idx, i in enumerate(self.x): self.xhat[idx, :] = soln[i:i+2]
        for idx, i in enumerate(self.l): self.lhat[idx, :] = soln[i:i+2]
        # above is MUCH better than these lines because it avoids reallocating
        # self.xhat = np.array([soln[i:i+2] for i in self.x])
        # self.lhat = np.array([soln[i:i+2] for i in self.l])