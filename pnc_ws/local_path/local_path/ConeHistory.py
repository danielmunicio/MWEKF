class ConeHistory:
    def __init__(self, max_size=1000):
        self.left_history = []  # Stores history of left cone points
        self.right_history = []  # Stores history of right cone points

    def update_history(self, left_points, right_points):
        """Update the history with new left and right points, trimming if necessary."""
        self.left_history.extend(left_points)  # Add the new points to the history
        self.right_history.extend(right_points)  # Add the new points to the history

    def get_history(self):
        """Return the history of left and right cone points."""
        return self.left_history, self.right_history
