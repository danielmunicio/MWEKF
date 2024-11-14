import numpy as np
import scipy as sp
from typing import Union

class GraphSLAMSolve:
    def __init__(self, x0: np.ndarray = np.array([0., 0.]), initial_rows: int = 100, initial_cols: int = 100, max_landmark_distance: float = 1, dx_weight: float = 2.0, z_weight: float = 1.0, dclip: float = 2.0, expansion_ratio: float = 1.7, local_radius: float = 1e5):
        """initialize GraphSLAMFast object

        Args:
            x0 (np.ndarray, optional): array [x, y] of initial state. Defaults to np.array([0.0, 0.0]).
            initial_rows (int, optional): initial number of equations this GraphSLAMFast can store. Will expand after this is reached. Defaults to 100.
            initial_cols (int, optional): initial number of variables this GraphSLAMFast can store. Will expand after this is reached. Defaults to 100.
            max_landmark_distance (float, optional): how far away landmarks can be from the closest landmark guess (AFTER data association optimally translates and rotates the landmarks) before they are recognized as independent. Defaults to 1.
            dx_weight (float, optional): weight (certainty) for odometry measurements. Defaults to 2.0.
            z_weight (float, optional): weight (certainty) for landmark measurements. Defaults to 1.0.
            dclip (float, optional): distance at which to clip cost function for data association. Defaults to 2.0.
            expansion_ratio (float, optional): amount by which to grow matrices when we run out of space. Defaults to 1.7.
            local radius (float, optional): radius to include cones in local map. Defaults to 1e5
        """
        self.max_landmark_distance = max_landmark_distance
        self.maxrows=initial_rows
        self.maxcols=initial_cols
        self.dx_weight = dx_weight # how much to weigh localization
        self.z_weight = z_weight # how much to weigh mapping
        self.local_radius = local_radius
        
        # Constraints for optimization problem, x here represents our optimal state vector representing the most likely values for all time steps & landmark positions
        #Ax-b = 0
        
        self.A = sp.sparse.lil_array((self.maxrows, self.maxcols), dtype=np.float64)
        self.b = np.zeros((self.maxcols), np.float64)

        #matrix initialized w these values
        #time step 0's x is x0[0]
        #time step 0's y is x0[1]

        self.A[0, 0] = 1
        self.A[1, 1] = 1
        self.b[0] = x0[0] # x
        self.b[1] = x0[1] # y

        self.nvars = 2
        self.neqns = 2

        self.x = [0]
        self.l = []

        self.z  = []
        self.d = [0]

        self.xhat = np.zeros((1, 2)) #running estimate of where car was at each time step
        self.lhat = np.zeros((0, 2)) #running estimate of where each landmark is
        self.color = np.zeros(0, dtype=np.uint8)
        self.xhat[0, :] = x0
        self.dclip = dclip


    def _transform(self, x: np.ndarray, center: np.ndarray, z: np.ndarray) -> np.ndarray:
        return (np.array([[np.cos(x[2]), -np.sin(x[2])],
                          [np.sin(x[2]), np.cos(x[2])]])@((z - center).T)).T + center + x[:2]
    
    def data_association(self, z: np.ndarray, color: np.ndarray) -> np.ndarray:
        """rudimentary data association method that tries to rotate and translate `z` until it optimally lines up with self.lhat

        Args:
            z (np.ndarray): measurements, in global reference frame
            color (np.ndarray): color of measurements

        Returns:
            np.ndarray: measurements, translated and rotated optimally
        """
        # center = np.mean(z, axis=0)
        center = self.xhat[-1, :]
        
        uniquecolors = filter(lambda x: np.any(self.color==x), np.unique(color))
        filters = [(self.lhat[self.color==c], color==c) for c in uniquecolors]

        # zprime = np.empty(z.shape)
        
        def cost(x: np.ndarray) -> float:
            """cost function for data association optimization. computes sum of squared distances from each measurement to each point, capped at self.dclip**2. This way further away landmarks don't have an effect.

            Args:
                x (np.ndarray): array of [x shift, y shift, rotation angle]

            Returns:
                float: cost of this x
            """
            zprime = self._transform(x, center, z)
            return sum(
                np.sum(
                    np.clip(
                        np.sum(np.square(zprime[filt, np.newaxis]-lhat), axis=2),
                        a_min=None, 
                        a_max=self.dclip**2,
                    )
                ) for (lhat, filt) in filters
            )

        res = sp.optimize.minimize(cost, x0=np.zeros(3), jac="2-point", options={"maxiter": 10}, tol=1e-3) 

        return self._transform(res.x, center, z)


    def data_assocation_lap_2(self, z: np.ndarray, color: np.ndarray) -> np.ndarray:
        """ use ICP for loop closure with every new set of cones in lap 2+
        
        Args:

        """
        pass

    #define line btwn orange cones method


    def lap_completion(self) -> bool:
        """ checks whether the car's latest position estimate crosses line between orange cone position estimate 
        
        Args: N/A - uses slam position data

        Updates:
            lap_counter: number of times car has passed orange cones

        Returns: N/A
        """
        pass


    
    def update_graph(self, dx: np.ndarray, z: np.ndarray, color: np.ndarray) -> None:
        """add edges to the graph corresponding to a movement and new vision data

        Args:
            dx (ndarray): difference in position from last update. [dx, dy]
            z (ndarray): measurements to landmarks. [[zx1, zy1], [zx2, zy2], ..., [zxn, zyn]]
            color (ndarray): categorical array of which color each of the measurements are. Elements should be dtype=np.uint8, or they'll be cast.
        """
        color = color.astype(np.uint8)
        # resize = (self.neqns, self.nvars)


        # Grow A and b matrix to account for # measurements and # variables >100 = initial_rows = initial_cols or updates beyond that
        cols = self.nvars + 2 + len(z)*2 > self.maxcols
        rows = self.neqns + 2 + len(z)*2 > self.maxrows
        if rows or cols:
            if cols: 
                self.maxcols = int(self.maxcols*1.5)
            if rows: 
                self.maxrows = int(self.maxrows*1.5)
                self.b = np.append(self.b, np.zeros(self.maxrows-len(self.b)))
                # self.b = np.pad(self.b, self.maxrows-len(self.b), constant_values=0.0, mode='constant')
                
            
            self.A.resize((self.maxrows, self.maxcols))
            
        # first add two equations and two variables
        # for the next position and the dx

        self.x.append(self.nvars)
        self.d.append(self.neqns)
        self.nvars += 2
        self.neqns += 2

        #localization update equations for new car position 

        #d[-1], d[-1]+1 are last two rows in A, representing x & y equations for the latest measurement
        #x[-1], x[-1] +1 are the last two columns in A, representing x& y variables for car at the 
        # last timestep, but why aren't the variables for car position just the first two in A

        self.A[self.d[-1], self.x[-1]] = 1 * self.dx_weight   
        self.A[self.d[-1]+1, self.x[-1]+1] = 1 * self.dx_weight
        self.A[self.d[-1], self.x[-2]] = -1 * self.dx_weight
        self.A[self.d[-1]+1, self.x[-2]+1] = -1 * self.dx_weight

        #

        self.b[self.d[-1]] = dx[0]*self.dx_weight
        self.b[self.d[-1]+1] = dx[1]*self.dx_weight

        # now add the guess for this position to xhat
        self.xhat = np.append(self.xhat, self.xhat[-1:, :] + dx, axis=0)


        # now do data association
        # to find the which landmarks correspond
        # to which measurements

        # first we try to rotate and translate z
        # so that it best lines up with the cones
        # we've already seen
        zprime = self.data_association(z+self.xhat[-1, :], color)
        # zprime = z+self.xhat[-1, :]
        # then we can check which cones are closest and which are within/outside
        # of the max landmark distance
        
        # dists has one row per measurement and one column for each landmark
        # can probably do this faster if we norm at the same time as broadcasting but can't do that easily
        for c in np.unique(color):
            z_c = zprime[color==c]
            if len(self.lhat[self.color==c])==0:
                l_idxs=np.zeros(len(z_c), dtype=int)
                l_dists=np.zeros(len(z_c))+np.Inf
            else:
                dists = np.linalg.norm(z_c[:, np.newaxis, :] - self.lhat[self.color==c], axis=2)
                l_idxs = np.argmin(dists, axis=1)
                l_dists = np.min(dists, axis=1)
            
            for i in range(len(z_c)):
                # if we haven't seen thihs landmark before, add it
                if l_dists[i] > self.max_landmark_distance:
                    self.l.append(self.nvars)
                    self.nvars += 2
                    self.lhat = np.append(self.lhat, z[color==c][i][np.newaxis], axis=0)
                    self.color = np.append(self.color, c)
                    l_idxs[i] = np.sum(self.color==c)-1 # len(self.l[self.color==c])-1  but self.l is a list so not bool mask indexable

                l = np.array(self.l)[self.color==c]
                # now we write the equation l-x=z
                self.z.append(self.neqns)
                self.neqns += 2
                self.A[self.z[-1], l[l_idxs[i]]]     = 1 * self.z_weight
                self.A[self.z[-1]+1, l[l_idxs[i]]+1] = 1* self.z_weight
                self.A[self.z[-1], self.x[-1]]       = -1 * self.z_weight
                self.A[self.z[-1]+1, self.x[-1]+1]   = -1 * self.z_weight
                self.b[self.z[-1]]                   = z[color==c][i, 0]*self.z_weight
                self.b[self.z[-1]+1]                 = z[color==c][i, 1]*self.z_weight
    
    def solve_graph(self) -> None:
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
        #* also turns out ufmpack breaks local_path for some reason? it has trouble linking to MA57 in codegen after installing
        #* libsuitesparse-dev, a dependency of scikit-ufmpack

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

    def get_cones(self, color: Union[np.uint8, None]) -> np.ndarray:
        if color is None: return self.lhat
        return self.lhat[self.color==color]
    def get_positions(self) -> np.ndarray:
        return self.xhat

