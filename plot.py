import matplotlib.pyplot as plt
import numpy as np

# Data points
left = np.array([[-2.72067306, 0.66813691],
                 [8.72374443, 3.04244087],
                 [20.42244796, 4.55462856],
                 [31.36371009, 4.33989006],
                 [33.52038388, -6.46645263],
                 [26.80795891, -15.7859027],
                 [16.30781263, -21.32591734],
                 [5.81788299, -26.83294197],
                 [-4.24758199, -22.41808789],
                 [-5.22303631, -10.90087683]])

right = np.array([[0.78133959, -2.69177246],
                  [9.75150149, -1.82131341],
                  [18.65985899, -0.87687368],
                  [27.2664268, 1.16447431],
                  [28.92707208, -6.65822958],
                  [22.84794767, -12.91842251],
                  [14.77296834, -16.97458042],
                  [6.92631664, -21.43672905],
                  [-0.46416783, -19.94343771],
                  [-0.72736326, -11.08695399]])

# Plot
plt.figure(figsize=(8, 6))
plt.plot(left[:, 0], left[:, 1], 'bo-', label='LEFT')  # Blue line with circles
plt.plot(right[:, 0], right[:, 1], 'ro-', label='RIGHT')  # Red line with circles

# Labels and legend
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("LEFT vs RIGHT Points")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()

