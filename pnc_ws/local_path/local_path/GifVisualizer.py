import matplotlib.pyplot as plt
import numpy as np
import imageio
import os

class GifVisualizer:
    def __init__(self, gif_filename: str = "cones_visualization.gif", fps: int = 10):
        self.gif_filename = gif_filename
        self.fps = fps
        self.frames = []
        
    def plot_frame(self, left_cones, right_cones, leftN_points, rightN_points, indices=None):
        """Plot one frame showing the cones and their indices."""
        fig, ax = plt.subplots(figsize=(8, 6))
        
        # Plot the left and right cones (raw data)
        ax.scatter(left_cones[:, 0], left_cones[:, 1], c='red', label='Left Cones')
        ax.scatter(right_cones[:, 0], right_cones[:, 1], c='blue', label='Right Cones')
        
        # Plot the ordered cones (lighter colors)
        ax.scatter(leftN_points[:, 0], leftN_points[:, 1], c='lightcoral', label='Ordered Left Cones')
        ax.scatter(rightN_points[:, 0], rightN_points[:, 1], c='lightblue', label='Ordered Right Cones')
        
        # Annotate the ordered cones with their indices
        if indices is not None:
            for i, point in enumerate(leftN_points):
                ax.annotate(str(indices[i]), (point[0], point[1]), textcoords="offset points", xytext=(0, 10), ha='center')
            for i, point in enumerate(rightN_points):
                ax.annotate(str(indices[i]), (point[0], point[1]), textcoords="offset points", xytext=(0, 10), ha='center')
        
        # Labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Cone Ordering Visualization')
        ax.legend(loc='upper right')
        
        # Capture the frame
        self.frames.append(self._save_frame(fig))
        plt.close(fig)
        
    def _save_frame(self, fig):
        """Convert a Matplotlib figure to a numpy array for GIF creation."""
        from io import BytesIO
        import matplotlib.pyplot as plt
        buf = BytesIO()
        fig.savefig(buf, format='png')
        buf.seek(0)
        img = imageio.imread(buf)
        return img
        
    def create_gif(self):
        """Create and save the GIF using the captured frames."""
        imageio.mimsave(self.gif_filename, self.frames, fps=self.fps)
    
    def update_gif(self, left_cones, right_cones, leftN_points, rightN_points, indices=None):
        """Update the GIF with new data."""
        self.frames = []  # Reset previous frames
        self.plot_frame(left_cones, right_cones, leftN_points, rightN_points, indices)
        self.create_gif()
        print("Updated GIF")
