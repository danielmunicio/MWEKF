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

    