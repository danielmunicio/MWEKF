#%%
from casadi import *
from .track import get_test_track, offset_path
import numpy as np
from numpy.linalg import *
from numpy.random import random
import matplotlib.pyplot as plt
from time import perf_counter






class GraphSLAM:
    sinc=lambda theta: 1-(theta**2)/6 + (theta**4)/120 - (theta**6)/5040
    cosc=lambda theta: -theta/2 + (theta**3)/24 - (theta**5)/720 + (theta**7)/40320
    def arc_to_global(dx, dth, pose): return np.array([[np.cos(pose[2]), -np.sin(pose[2])],
                                                       [np.sin(pose[2]),  np.cos(pose[2])],
                                                       [            0.0,              0.0]])@(dx*([GraphSLAM.sinc(dth), 
                                                                                                   GraphSLAM.cosc(dth)])) + pose + np.array([0, 0, dth])
    
    def polar_to_global(r, theta, pose): return (r+pose[2])*np.array([np.cos(theta), np.sin(theta)]) + pose[:2]
    
    def __init__(self, **settings):
        # start position of the car
        self.x0 = np.array([0,0])

        # flag to indicate whether we can just add all the 
        # landmarks we see (on the first iteration)
        self.firstupdate = True
        
        # how far a landmark can be from another while still
        # being considered the same landmark
        self.landmarkTolerance = 2
        
        self.solver_type = 'nlp' # either 'qp' or 'nlp'
        # options for IPOPT
        # first three control printing
        # last sets linear solver
        # if you don't have the HSL solvers,
        # comment out the line and it'll use
        # MUMPS, which comes with CasADi
        # 'qp' section is options for osqp
        # if we're using that instead of IPOPT
        # bc sometimes its faster
        self.solver_opts = {
            'nlp':{
                'ipopt.print_level': 0,
                'ipopt.sb': 'yes',
                'print_time': 0,
                # 'ipopt.linear_solver': 'MA27', # uncomment if you have HSL stuff installed
            },
            'qp':{
                'print_time': 0,
                'print_problem': 0,
            }
        }

        # weight matrices describing the quadratic forms
        # in the cost functions
        # for example, if you want to increase the significance 
        # of errors on the first two cones - perhaps to help 
        # with loop closing - you could set R's first four 
        # (for 2 x and 2 y) diagonal elements to be higher than 
        # the rest.
        self.Q = lambda n: DM_eye(n) # cost for pose edges
        self.R = lambda n: DM_eye(n) # cost for landmark edges
        self.C = lambda n: DM.eye(n) # cost for landmark color

        # so people can pass in settings as kw args
        # just me being lazy and not writing out keyword argguments
        # in the function declaration
        # but you could pass in, for example, landmarkTolerance=2,
        # and it would get updated here
        self.__dict__.update(settings)

        # these things are after since they shouldn't be settings

        # symbolic position variables (odo nodes)
        self.x = [MX.sym('x0', 2)] # initialize with first position
        # bad guess for position nodes. Used as initial values for optimization.
        self.xhat = [self.x0]
        # list of odo edges: the equations x_(k) - x_(k-1) = z_k (z_k is odo measurement)
        self.x_edges = []

        # now samee thing for landmarks
        # symbolic landmark positions (landmark nodes)
        self.lm = []
        # bad guess for landmark positions. Used as initial values for optimization and also data association.
        self.lmhat = []
        # list of landmark edges: the equations lm_(k)i - x_k = z_(k)i (z_k is list of landmark measurements)
        self.lm_edges = []
        #list of colors for each landmark
        self.colors = []

        self.last_loc = [0,0]
        self.state_backlog = []
        self.perception_backlog_cones = []
        self.perception_backlog_pos = []

    def update_position(self, dx):
        """updates graph given only odometry, used between landmark measurement updates

            In practice, this only does the first half of the update_graph function

        Args: 
            dx (ndarray): vector of shape (2, 1) describing the estimated change in car location since the previous update
        """
        # guess current position
        # by adding dx to previous position guess
        self.xhat.append(self.xhat[-1] + dx)
        curpos = self.xhat[-1] # useful later for succinctness
        
        # add pose node (add symbolic position variable to list)
        self.x.append(MX.sym(f'x{len(self.x)}', 2))
        # add corresponding pose edge.
        # equation is x[-2] + odo_measurement = x[-1]
        # so we rearrange into a minimization problem
        # x[-2] + odo_measurement - x[-1] = 0
        self.x_edges.append((self.x[-2]+DM(dx)-self.x[-1]))
    
    def update_backlog_imu(self, dx):
        self.last_loc += dx 
        self.state_backlog += [last_loc]
    
    def update_backlog_perception(self, z):
        self.perception_backlog_cones += [z]
        self.perception_backlog_pos += [self.last_loc]

    def update_graph_block(self): #update_graph over multple time steps
        """updates graph given odo and lm measurements

        Args:
            dx (ndarray): vector of shape (2, 1) describing the estimated change in car location since the previous update 
            z (ndarray): vector of shape (m, 3*) describing locations of m landmarks relative to the car #* updated 2 to 3 to incorporate color - Rohan
            update_pos (boolean): whether to update position in this method (used to ignore dx for ros integration)
        """ 
        for i in range(len(self.state_backlog)):
            self.x.append(MX.sym(f'x{len(self.x)}', 2))
            self.x_edges.append((self.x[-2]+DM(dx)-self.x[-1]))

        n = len(self.perception_backlog_cones)
        for t in range(n): 
            #zcoords = z[:,:2]
            curpos = self.perception_backlog_pos[t] # useful later for succinctness
            curpos_color = np.append(curpos, [0])
            pbc = self.perception_backlog_cones[t]
            t_dx = pbc[0] * np.cos(pbc[1])
            t_dy = pbc[0] * np.sin(pbc[1])
            z = np.array([t_dx, t_dy, pbc[2]])
            

            # if its the first graph update, we do things a bit differently
            if len(self.lmhat)==0:
                # set landmark guesses to absolute location of landmarks (add measurements to current pose)
                self.lmhat = z + curpos_color # color
                #self.lmhat = z+curpos # no color
                # add landmark nodes (add symbolic landmark nodes)
                #self.lm = [MX.sym(f'lm{i}', 2) for i in range(z.shape[0])] # no color
                
                self.lm = [MX.sym(f'lm{i}', 3) for i in range(z.shape[0])]      #note: think second sym parameter represents length of first param
                # add corresponding landmark edges. Same equation as for the pose edges
                self.lm_edges = [self.x[-1] + DM(z[i][:2]) - self.lm[i][:2] for i in range(z.shape[0])] 

                self.colors = [DM(z[i][2]) - self.lm[i][2] for i in range(z.shape[0])] #associate colors by adding color binding constraint, equation: seen_color - landmark_color_guess = 0
                
                # now return so we don't add these landmarks twice
                return
            

            # for each landmark measurement:
            for i in z:

                idx = np.argmin(dists:=norm(self.lmhat[:,:2]-(i[:2] + curpos), axis=1)) # do colorblind data association but keep color constraints 
                #i is landmark guess from lidar, idx is index of best guess among current landmarks
                if dists[idx] > self.landmarkTolerance:
                    # if it's too far, we should add this as a new landmark
                    idx = len(self.lmhat) # set idx to the element after the current last one
                    # append the measured landmark's approximate position to our list of landmark position guesses
                    self.lmhat = np.concatenate((self.lmhat, (i + curpos_color)[np.newaxis]), axis=0) # color
                    # finally, append a symbolic variable to the list of landmark nodes to represent this landmark's position
                    self.lm.append(MX.sym(f'lm{idx}', 3))
                self.colors.append(i[2] - self.lm[idx][2]) 
                # Now that we know the landmark's symbolic variable's index, we can add an edge!
                # Equation is the same as before.
                self.lm_edges.append((self.x[-1] + DM(i[:2]) - self.lm[idx][:2])) # x + z_i = lm_i
        self.state_backlog = []
        self.perception_backlog_cones = []
        self.perception_backlog_pos = []



    def update_graph(self, dx, z):
        """updates graph given odo and lm measurements

        Args:
            dx (ndarray): vector of shape (2, 1) describing the estimated change in car location since the previous update 
            z (ndarray): vector of shape (m, 2*) describing locations of m landmarks relative to the car
        """
        # guess current position
        # by adding dx to previous position guess
        self.xhat.append(self.xhat[-1] + dx)
        curpos = self.xhat[-1] # useful later for succinctness
        curpos_color = np.append(curpos, [0]) # add 0 for comparison with landmarks with extra color attribute -rohan
        
        # add pose node (add symbolic position variable to list)
        self.x.append(MX.sym(f'x{len(self.x)}', 2)) 
        # add corresponding pose edge.
        # equation is x[-2] + odo_measurement = x[-1]
        # so we rearrange into a minimization problem
        # x[-2] + odo_measurement - x[-1] = 0
        self.x_edges.append((self.x[-2]+DM(dx)-self.x[-1]))

        # if its the first graph update, we do things a bit differently
        if len(self.lmhat) == 0:
            # set landmark guesses to absolute location of landmarks (add measurements to current pose)
            #self.lmhat = z+curpos_color # color
            self.lmhat = z+curpos # no color
            # add landmark nodes (add symbolic landmark nodes)
            #self.lm = [MX.sym(f'lm{i}', 3) for i in range(z.shape[0])] # color
            self.lm = [MX.sym(f'lm{i}', 2) for i in range(z.shape[0])]      #note: i think second sym parameter represents length of first param idk
            # add corresponding landmark edges. Same equation as for the pose edges
            #self.lm_edges = [self.x[-1] + DM(z[i][:2]) - self.lm[i][:2] for i in range(z.shape[0])] # color
            self.lm_edges = [self.x[-1] + DM(z[i]) - self.lm[i] for i in range(z.shape[0])] # no color
            # now return so we don't add these landmarks twice
            return
        
        # data association and adding edges

        # for each landmark measurement:
        for i in z:
            # find the index of the closest landmark guess we have so far
            # this index is the same as the index of the landmark's symbolic position variable
            # in self.lm
            # this is the part that breaks if lmhat is empty. since np won't broadcast i+curpos to length zero
            #lm_color = i[2]
            #lmhat_color = self.lmhat[self.lmhat[2] == lm_color] # only compare seen landmark with known landmarks of the same color
            idx = np.argmin(dists:=norm(self.lmhat-(i+curpos), axis=1)) # no cone color
            #idx = np.argmin(dists:=norm(lmhat_color[:2]-(i[:2]+curpos), axis=1)) # cone color

            # check if the landmark we found is close enough to the measurement to be considered
            # the samee laandmark

            #i is landmark guess from lidar, idx is index of best guess among current landmarks
            if dists[idx] > self.landmarkTolerance:
                # if it's too far, we should add this as a new landmark
                idx = len(self.lmhat) # set idx to the element after the current last one
                # append the measured landmark's approximate position to our list of landmark position guesses
                #self.lmhat = np.concatenate((self.lmhat, (i + curpos_color)[np.newaxis]), axis=0) # color
                self.lmhat = np.concatenate((self.lmhat, (i + curpos)[np.newaxis]), axis=0) # no color
                #self.lmhat = np.concatenate((self.lmhat, (i + curpos_color)[np.newaxis]), axis=0) 
                # finally, append a symbolic variable to the list of landmark nodes to represent this landmark's position
                self.lm.append(MX.sym(f'lm{idx}', 2))
            # Now that we know the landmark's symbolic variable's index, we can add an edge!
            # Equation is the same as before.
            self.lm_edges.append((self.x[-1] + DM(i) - self.lm[idx])) # x + z_i = lm_i, no color
            #self.lm_edges.append((self.x[-1] + DM(i[:2]) - self.lm[idx][:2])) # x + z_i = lm_i


    def update_graph_color(self, z): #update_graph but w color - rohan
        """updates graph given odo and lm measurements

        Args:
            dx (ndarray): vector of shape (2, 1) describing the estimated change in car location since the previous update 
            z (ndarray): vector of shape (m, 3*) describing locations of m landmarks relative to the car #* updated 2 to 3 to incorporate color - Rohan
            update_pos (boolean): whether to update position in this method (used to ignore dx for ros integration)
        """ 
        #zcoords = z[:,:2]
        curpos = self.xhat[-1] # useful later for succinctness
        curpos_color = np.append(curpos, [0])

        # if its the first graph update, we do things a bit differently
        if len(self.lmhat)==0:
            # set landmark guesses to absolute location of landmarks (add measurements to current pose)
            self.lmhat = z+curpos_color # color
            #self.lmhat = z+curpos # no color
            # add landmark nodes (add symbolic landmark nodes)
            #self.lm = [MX.sym(f'lm{i}', 2) for i in range(z.shape[0])] # no color
            
            self.lm = [MX.sym(f'lm{i}', 3) for i in range(z.shape[0])]      #note: think second sym parameter represents length of first param
            # add corresponding landmark edges. Same equation as for the pose edges
            self.lm_edges = [self.x[-1] + DM(z[i][:2]) - self.lm[i][:2] for i in range(z.shape[0])] 

            self.colors = [DM(z[i][2]) - self.lm[i][2] for i in range(z.shape[0])] #associate colors by adding color binding constraint, equation: seen_color - landmark_color_guess = 0
            
            # now return so we don't add these landmarks twice
            return
        
        # data association and adding edges

        # for each landmark measurement:
        for i in z:
            # find the index of the closest landmark guess we have so far
            # this index is the same as the index of the landmark's symbolic position variable
            # in self.lm
            # this is the part that breaks if lmhat is empty. since np won't broadcast i+curpos to length zero

            # tried making alg only compare landmarks of the same color but letting optimization alg do it naturally was simpler 
            #lm_color = i[2] 
            #lmhat_color = self.lmhat[self.lmhat[2] == lm_color] 
            #idx = np.argmin(dists:=norm(lmhat_color[:2]-(i[:2] + curpos) + lmhat, axis=1)) # cone color
            
            idx = np.argmin(dists:=norm(self.lmhat[:,:2]-(i[:2] + curpos), axis=1)) # do colorblind data association but keep color constraints 
            #biglargenum = 1000000
            #idx = np.argmin(dists:=norm(self.lmhat[:,:2]-(i[:2] + curpos), axis=1) + biglargenum * (self.lmhat[:,2] - i[2])) # cone color method 2, add extra weight to ensure colors r same ** DOESNT WORK **


            # check if the landmark we found is close enough to the measurement to be considered
            # the samee laandmark

            #i is landmark guess from lidar, idx is index of best guess among current landmarks
            if dists[idx] > self.landmarkTolerance:
                # if it's too far, we should add this as a new landmark
                idx = len(self.lmhat) # set idx to the element after the current last one
                # append the measured landmark's approximate position to our list of landmark position guesses
                self.lmhat = np.concatenate((self.lmhat, (i + curpos_color)[np.newaxis]), axis=0) # color
                # finally, append a symbolic variable to the list of landmark nodes to represent this landmark's position
                self.lm.append(MX.sym(f'lm{idx}', 3))
            self.colors.append(i[2] - self.lm[idx][2]) 
            # Now that we know the landmark's symbolic variable's index, we can add an edge!
            # Equation is the same as before.
            self.lm_edges.append((self.x[-1] + DM(i[:2]) - self.lm[idx][:2])) # x + z_i = lm_i
    
    


        
    def solve_graph(self):
        """solves the graph and updates everything
        """
        # step 1: put all the equations together in a big list
        # vertcat flattens things, so we'll get a list of x1, y1, x2, y2, x3, y4, etc. etc.
        x_e = vertcat(*self.x_edges) # all pose edges
        lm_e = vertcat(*self.lm_edges) # all landmark edges
        x0_e = self.x[0] - DM(self.x0) # an extra edge asserting that x0 should be pinned down (otherwise the problem is underdetermined and x0 and y0 are free variables)
        lm_color = vertcat(*self.colors)
        
        colorsolve = True
        lm_dim = 3 if colorsolve else 2
        
        # construct the QP
        # x is the optimization variable
        # f is the objective function
        qp = {
            'x':vertcat(*self.x, *self.lm), # just all of the pose and landmark nodes
            # construct the cost function in threee parts:
            # 1. pose quadratic
            # 2. landmark quadratic
            # 3. x0 constraint
            # 4. color constraint - Rohan
            'f': (x_e.T@self.Q(x_e.shape[0])@x_e # note: j norm squared error, q,r, and c can be adjusted above for weighting - rohan
                + lm_e.T@self.R(lm_e.shape[0])@lm_e 
                + x0_e.T@x0_e
                + lm_color.T@self.C(lm_color.shape[0])@lm_color) # color constraint
        }

        # so now we create the solver
        # qpsol *should* be ideal, since we have a quadratic problem,
        # however just using ipopt sometimes works better
        # on windows ipopt + ma27 was faster, but on linux, osqp is faster.

        if self.solver_type == 'qp':
            solver = qpsol('solver', 'osqp', qp, self.solver_opts[self.solver_type])
        elif self.solver_type == 'nlp':
            solver = nlpsol('solver', 'ipopt', qp, self.solver_opts[self.solver_type])
        else:
            assert f"solver_type must be 'qp' or 'nlp', not {self.solver_type}"

        # actually solve the QP problem
        soln = np.array(solver(x0=vertcat(*self.xhat, *self.lmhat))['x'])

        # now we need to get the solution back into a good form
        # first find split between pose nodes and landmark nodes
        split = len(self.xhat)*2

        # desired shape of the pose solutions
        s = (len(self.xhat), 2)
        # get the slice and reshape it
        x_guesses = np.array(soln[:split])
        x_guesses.resize(s)

        # now do the same thing for the landmark nodes
        s = (len(self.lmhat), lm_dim) #2 for no cone color, 3 for cone color
        lm_guesses = np.array(soln[split:])
        lm_guesses.resize(s)

        # now update self.xhat and self.lmhat with our newly optimized
        # guesses. This assumes that no new nodes have been added during the optimization.
        self.xhat[:x_guesses.shape[0]] = list(x_guesses)
        #self.lmhat[:lm_guesses.shape[0]] = list(lm_guesses)
        self.lmhat[:,:lm_dim] = list(lm_guesses) #2 for no cone color, 3 for cone color
        #***if color***
        #self.lmhat[:lm_guesses.shape[0]]
        
        
        # return the solutions in case they're useful for debugging
        return list(x_guesses), list(lm_guesses)
    
########################################################################################################################################################################################################################

def get_path():
    path = get_test_track()
    path = np.array([path[i] for i in range(path.shape[0]) if i%3==0])
    # path = path-path[0]
    left = offset_path(path, 4)
    right = offset_path(path, -4)
    left = np.array([left[i] for i in range(path.shape[0]) if i%2==0])
    right = np.array([right[i] for i in range(path.shape[0]) if i%2==0])
    return path, left, right
path, left, right = get_path()
#%%
class Sim:
    CARTESIAN = 1
    POLAR = 2
    def __init__(self, path, width, odo_noise_func, measurement_noise_func, vision_range=10, pose_divider=2, lm_divider=3, shift_to_origin=True, record_no_slam=True, mode=CARTESIAN):
        self.path = path
        self.width = width
        self.pose_divider = pose_divider
        self.lm_divider = lm_divider
        self.shift_to_origin = shift_to_origin
        self.mode=mode
        self.idx = 0
        self.dx_noise = odo_noise_func
        self.z_noise = measurement_noise_func
        self.vision_range = vision_range
        self.record_no_slam = record_no_slam
        self.no_slam_x = [self.path[0]]
        self.no_slam_m = []
        self._update()
    def _update(self):
        """util to run whenever settings are changed
        """
        if self.shift_to_origin:
            self.path -= self.path[0]
        self.x = np.array([self.path[i] for i in range(self.path.shape[0]) if i%self.pose_divider==0])
        self.left = offset_path(self.path, self.width/2)
        self.right = offset_path(self.path, -self.width/2)
        self.left = np.array([self.left[i] for i in range(self.left.shape[0]) if i%self.lm_divider==0])
        self.right = np.array([self.right[i] for i in range(self.right.shape[0]) if i%self.lm_divider==0])
    def rot(self, a):
        return np.array([[np.cos(a), -np.sin(a)],[np.sin(a), np.cos(a)]])
    def get_step(self):
        # get fake observations
        z = np.concatenate(
            ((self.left[norm(self.left-self.path[self.idx], axis=1)<self.vision_range]), # change when creating an actual sim class
            (self.right[norm(self.right-self.path[self.idx], axis=1)<self.vision_range])),
            axis=0
        )-self.path[self.idx]
        # plt.scatter(*(z+self.path[self.idx]).T)
        # plt.show()
        dx = self.path[self.idx]-self.path[self.idx-1]
        dx_noise = self.dx_noise((2,))
        z_noise = self.z_noise(z.shape)
        if self.record_no_slam:
            dx = self.path[self.idx]-self.path[self.idx-1]
            self.no_slam_x.append(self.no_slam_x[-1]+dx+dx_noise)
            self.no_slam_m.append(z+self.no_slam_x[-1]+z_noise)

        if self.mode == Sim.POLAR:
            dx = self.path[self.idx]-self.path[self.idx-1]
            dx_prev = self.path[self.idx-1]-self.path[self.idx-2]
            dx_out = np.array([norm(dx), (1 if det(np.concatenate((dx_prev[:, np.newaxis], dx[:, np.newaxis]), axis=1))>0 else -1)*np.arccos(dx@dx_prev/(norm(dx)*norm(dx_prev)))]) # convert to polar
            
            z = (self.rot(-(1 if det(np.concatenate((np.array([[1],[0]]), dx[:, np.newaxis]), axis=1))>0 else -1)*np.arccos(dx@np.array([1,0])/norm(dx)))@(z.T)).T
            norm_z = z/norm(z, axis=1, keepdims=True)
            
            z = np.concatenate((norm(z, axis=1, keepdims=True), (np.sign([det(np.concatenate((np.array([[1],[0]]),norm_z[i][:, np.newaxis]), axis=1)) for i in range(norm_z.shape[0])])*np.arccos(norm_z[:, 0]))[:, np.newaxis]), axis=1)
            dx = dx_out
        self.idx += 1
        return dx+dx_noise, z+z_noise
    def __iter__(self):
        self.idx=0
        self.no_slam_x = [self.path[0]]
        self.no_slam_m = []
        self._update()
        return self
    def __next__(self):
        if self.idx==len(self.path):
            raise StopIteration
        out = self.get_step()
        return out


def plot_results(slam, sim, solve_times):
    x_guesses, lm_guesses = slam.xhat, slam.lmhat
    no_slam_x, no_slam_lm = sim.no_slam_x, sim.no_slam_m

    fig, axs = plt.subplots(2, 2, gridspec_kw=dict(height_ratios=[3,1]))

    # plot true track
    for ax in axs[0]:
        ax.plot(*sim.path.T, color='tab:blue', linestyle='dashed')
        ax.plot(*sim.left.T, color='tab:blue', label='track')
        ax.plot(*sim.right.T, color='tab:blue')


    axs[0][0].plot(*np.array(x_guesses).T, color='red', label='poses')
    axs[0][0].scatter(*np.array(lm_guesses).T, color='green', label='landmarks', s=4)
    axs[0][0].set_title('Localization with GraphSLAM')

    axs[0][1].plot(*np.array(no_slam_x).T, color='red', linestyle='dashed', label='slam input path')
    axs[0][1].scatter(*np.concatenate(no_slam_lm, axis=0).T, color='green', s=4)
    axs[0][1].set_title('Localization without GraphSLAM')

    axs[1][0].plot(np.array(solve_times)[:, 1], label='solve time')
    axs[1][0].set_xlabel('Drive Distance (2m)')
    axs[1][0].set_ylabel('Time (s)')
    axs[1][0].set_title('Solve Time vs. Drive Distance')

    axs[1][1].plot(*np.array(solve_times).T, label='solve time')
    axs[1][1].set_xlabel('number of edges')
    axs[1][1].set_ylabel('Time (s)')
    axs[1][1].set_title('Solve Time vs. Number of Edges')

    fig.set_tight_layout(True)
    # fig.suptitle(f"GraphSLAM: re-optimization at each new pose\nLandmark view distance: {vision_range}m, Solver: IPOPT+MUMPS")
    fig.suptitle(f"GraphSLAM: single optimization after completed lap.\nLandmark view distance: {sim.vision_range}m, Solver: {'IPOPT' if slam.solver_type=='nlp' else 'OSQP'}. Final solve time: {solve_times[-1][1]}s")
    plt.show()


#%%

# testing
# these two `if __name__ == '__main__':` blocks *should* do exactly the same thing,
# but the first one has the entire output shifted very slightly in the positive x direction.
# this means I think it's a simulation or plotting issue, so it's not a real problem, but
# i'm keeping the second if statement in case i need nice visuals.
if __name__ == '__main__' and True:
    sim = Sim(
        path, 
        8,
        odo_noise_func=lambda shape: np.random.random(shape)*0.1, 
        measurement_noise_func=lambda shape: np.random.random(shape)*1,
        pose_divider=3,
        lm_divider=2,
        shift_to_origin=False,
        mode=Sim.CARTESIAN,
    )
    slam = GraphSLAM(x0=path[0], solver_type='qp')
    i = 0
    solve_times = []
    for dx, z in sim:
        # for timing
        tic = perf_counter()

        # actual update and solve
        slam.update_graph(dx, z)
        slam.solve_graph()

        # timing stuff
        toc = perf_counter()
        solve_times.append([len(slam.x_edges)+len(slam.lm_edges), toc-tic])

        # progress bar stuff
        barlen = 20
        print(f'\r|{"@"*int(np.ceil(barlen*(i+1)/len(path)))}{"."*(barlen-int(np.floor(barlen*(i+1)/len(path))))}| {i+1}/{len(path)}', end='')
        i += 1
    print()
    plot_results(slam, sim, solve_times)
    exit()


if __name__ == '__main__':
    no_slam_x = [path[0]]
    no_slam_lm = []
    solve_times = []
    vision_range = 10
    slam = GraphSLAM(x0=path[0], solver_type='qp')
    for i in range(1, len(path)):
        # get fake odometry measurement
        dx = (path[i]-path[i-1]+(random(2)*0.1)) # add noise with stdev 0.01
        no_slam_x.append(no_slam_x[-1]+dx)
        # get fake observations
        z = np.concatenate(
            ((left[norm(left-path[i], axis=1)<vision_range]), # change when creating an actual sim class
            (right[norm(right-path[i], axis=1)<vision_range])),
            axis=0
        )-path[i]
        z = z+random(z.shape)*1
        no_slam_lm.append(z+no_slam_x[-1])

        # update graph
        t0 = perf_counter()
        slam.update_graph(dx, z)
        t1 = perf_counter()
        #* comment this line out to only solve at the end
        x_guesses, lm_guesses = slam.solve_graph()
        t2 = perf_counter()
        solve_times.append([t1-t0, t2-t1, len(slam.x_edges)+len(slam.lm_edges)])
        # progress bar
        barlen = 20
        print(f'\r|{"@"*int(np.ceil(barlen*(i+1)/len(path)))}{"."*(barlen-int(np.floor(barlen*(i+1)/len(path))))}| {i+1}/{len(path)}', end='')
    print()
    # %%
    # plt.style.use('dark_background')
    fig, axs = plt.subplots(2, 2, gridspec_kw=dict(height_ratios=[3,1]))

    # plot true track
    for ax in axs[0]:
        ax.plot(*path.T, color='tab:blue', linestyle='dashed')
        ax.plot(*left.T, color='tab:blue', label='track')
        ax.plot(*right.T, color='tab:blue')


    axs[0][0].plot(*np.array(x_guesses).T, color='red', label='poses')
    axs[0][0].scatter(*np.array(lm_guesses).T, color='green', label='landmarks', s=4)
    axs[0][0].set_title('Localization with GraphSLAM')

    axs[0][1].plot(*np.array(no_slam_x).T, color='red', linestyle='dashed', label='slam input path')
    axs[0][1].scatter(*np.concatenate(no_slam_lm, axis=0).T, color='green', s=4)
    axs[0][1].set_title('Localization without GraphSLAM')

    axs[1][0].plot(np.array(solve_times)[:, 1], label='solve time')
    axs[1][0].set_xlabel('Drive Distance (2m)')
    axs[1][0].set_ylabel('Time (s)')
    axs[1][0].set_title('Solve Time vs. Drive Distance')

    axs[1][1].plot(*np.array(solve_times)[:, 2:0:-1].T, label='solve time')
    axs[1][1].set_xlabel('number of edges')
    axs[1][1].set_ylabel('Time (s)')
    axs[1][1].set_title('Solve Time vs. Number of Edges')

    fig.set_tight_layout(True)
    # fig.suptitle(f"GraphSLAM: re-optimization at each new pose\nLandmark view distance: {vision_range}m, Solver: IPOPT+MUMPS")
    fig.suptitle(f"GraphSLAM: single optimization after completed lap.\nLandmark view distance: {vision_range}m, Solver: IPOPT+MUMPS. Solve time: 0.1223s")
    plt.show()