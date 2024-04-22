import numpy as np
import math

N = 30

# Time step
t = 0.01

# Simulated noise standard deviation of observation 
# In this case, we need this to simulate the LiDar Covariance of measurement
STANDARD_DEVIATION_NOISE_D = 0.05
STANDARD_DEVIATION_NOISE_THETA = np.deg2rad(1)

# Standard deviation of gaussian for resampling
STANDARD_DEVIATION_XY = 0.25
STANDARD_DEVIATION_THETA = np.deg2rad(5)

Q = np.diag([STANDARD_DEVIATION_NOISE_D * 3, STANDARD_DEVIATION_NOISE_THETA * 3]) ** 2

# Resampling max weight limit
RESAMPLING_MAX_LIMIT = 0.01
# Minimum weight of particle
W_MIN = 0.01

class Particle:
    # Particle for vehicle location and heading
    def __init__(self, x, y, w, theta):
        self.x = x
        self.y = y # location of particle
        self.w = w # weight of particle
        self.theta = theta # heading of vehicle

    
    def __str__(self) -> str:
        print(f"x: {self.x} \n" + 
              f"y: {self.y} \n" +
              f"w: {self.w} \n" +
              f"theta: {self.theta} \n")

    



class FastSLAM:
    """
    Constructor:
    - To be called upon finishing the map and path
    - will be initialized with a completed map and the initial location of the vehicle
    
    """



    def __init__(self, map_of_cones: list, initial_loc: tuple):
        self.particles = [Particle(initial_loc[0], initial_loc[1], 1 / N, initial_loc[2]) for _ in range(N)]
        self.map_of_cones = map_of_cones
    """
    Main Function
    - Start with a list of particles
    - For each particle, move them in the direction of the dx
    - Update the weight of the particle based on the cone observations
    - Resample Particles
    - Return highest weighted particle


    Inputs;
    - particles: list of particles
    - dx: np.array of the change in x, y, and theta respectively
    - z: list of obs_cones (r, theta, color)

    Outputs:
    - list of particles

    #NOTE: This won't really be run in practice because we can't expect to have the observations and the cone data at the same time
    """
    def fastslamit(self, dx: tuple, z: list) -> list[Particle]:
        # Move particles
        # dx is in the form [dx, dy, dtheta]
        # may have to be processed into this form from imu data depending on the node itself
        self.predict_poses(dx)

        self.particles = self.update_weights(z)
        # Return all particles
        return self.particles

    def update_weights(self, z: list) -> list[Particle]:
        # Update weights
        # For each particle, perform data association with the cone and then update weight based on that cone
        for particle in self.particles:
            for obs_cone in z:
                # convert from polar to cartesian from perspective of this particle
                obs_cone_x = particle.x + obs_cone[0] * math.cos(particle.theta + obs_cone[1])
                obs_cone_y = particle.y + obs_cone[0] * math.sin(particle.theta + obs_cone[1])
                obs_cone_cart = (obs_cone_x, obs_cone_y, obs_cone[2])

                # Perform data association using the cartesian measurements
                closest_cone = self.data_assoc(obs_cone_cart)
                # Update weight
                # closest_cone = expected observation
                # obs_cone = actual observation
                
                # obs_cone is in correct r theta format
                # we need to put closest_cone into correct r theta format
                x_closest = closest_cone[0] - closest_cone[0]
                y_closest = closest_cone[1] - closest_cone[1]
                r = np.sqrt(x_closest**2 + y_closest**2)
                theta = np.arctan2(y_closest, x_closest) - particle.theta
                closest_cone_polar = (r, theta, closest_cone[2])
                # Now, both obs cone and closest cone polar will be in the desired format
                #NOTE: Confirm whether heading angle should be in degrees or radians
                particle.w *= self.calculate_weight(Q, obs_cone, closest_cone_polar)


        # Resample particles
        self.particles = self.resample_particles(N, W_MIN)
        # Return all particles
        return self.particles



    def predict_poses(self, dx: tuple) -> list[Particle]:
        for particle in self.particles:
            # Add noise to add variance to particles
            particle.x += dx[0]
            particle.y += dx[1]
            particle.theta += dx[2]
        return self.particles


    """
    Function that finds the closest cone to the observed cone given that it is the same color
    Inputs:
    - list_of_cones: list of cones (x, y, color)
    - obs_cone: observed cone (x, y, color)
    Output:
    - closest_cone: closest cone to the observed cone

    """
    def data_assoc(self, obs_cone: tuple) -> tuple:
        # Find the closest cone in list_of_cones to the obs_cone given they are the same color
        closest_cone = None
        min_distance = float('inf')
        for cone in self.map_of_cones:
            if cone[2] == obs_cone[2]:
                distance = np.linalg.norm(np.array(cone[:2]) - np.array(obs_cone[:2]))
                if distance < min_distance:
                    closest_cone = cone
                    min_distance = distance
        return closest_cone

    """
    Calculates Weights
    Inputs:
    - Q: Measurement covariance
    - obs_cone: Actual observation (r, theta, color)
    - closest_cone: Expected observation (x, y, color)

    """

    def calculate_weight(self, Q, obs_cone, closest_cone):

        diff = np.subtract(obs_cone, closest_cone)
        w = ( np.linalg.det(np.multiply(2 * np.pi, Q)) ** -0.5 ) * ( np.exp((-0.5) * np.transpose(diff) @ np.linalg.inv(Q) @ diff) )
        return w

    # Sample Proposal Func
    # particles is list of particles
    # w_min is the threshold value for which particles will be discarded
    def resample_particles(self, num_particles, w_min):
        # TODO: this could be a poor resampling procedure, may need to find/choose a better one

        # create an empty list to hold the particles that we will return
        nu_particles = []

        particles_deleted = 0

        # Normalize weights of particles
        particles = self.normalize_weights(self.particles)

        # Backup particle incase all particles are deleted
        heightest_wegit_particle = self.get_heightest_wegit(particles)

        # Loop through all particles
        for i in range(0, num_particles):

            # Check that particles is greater than min threshold and sample with random probability
            if particles[i].w > w_min and particles[i].w > np.random.random() * RESAMPLING_MAX_LIMIT:
                nu_particles.append(particles[i])
            else:
                particles_deleted += 1
        
        # Get new highest weight particle
        if len(nu_particles) > 0:
            heightest_wegit_particle = self.get_heightest_wegit(nu_particles)
        
        # Replenish deleted particles based on highest weight particle
        x, y, theta = heightest_wegit_particle.x, heightest_wegit_particle.y, heightest_wegit_particle.theta
        for i in range(0, particles_deleted):
            x_new = np.random.normal(x, STANDARD_DEVIATION_XY)
            y_new = np.random.normal(y, STANDARD_DEVIATION_XY)
            theta_new = np.random.normal(theta, STANDARD_DEVIATION_THETA)
            replacement_particle = Particle(x_new, y_new, 1/num_particles, theta_new)

            replacement_particle.lm = heightest_wegit_particle.copytree()

            nu_particles.append(replacement_particle)

        return self.normalize_weights(nu_particles)


    # Normalize weights so they sum up to 1
    def normalize_weights(self, particles):
        total = 0
        # Get total weight
        for particle in particles:
            total += particle.w
        
        # Normalize weight from 0 - 1
        if total != 0:
            for particle in particles:
                particle.w = particle.w/total
        # Set to default weight if all weights are 0
        else:
            for particle in particles:
                particle.w = 1/len(particles)
        
        return particles

    # Returns highest weight particle
    def get_heightest_wegit(self, particles):
        return max(particles, key=lambda x : x.w)