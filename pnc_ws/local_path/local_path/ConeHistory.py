import json
import os

class ConeHistory:
    def __init__(self, max_size=1000, file_name="cone_history.json"):
        self.left_history = []  # Stores history of left cone points
        self.right_history = []  # Stores history of right cone points
        self.file_name = file_name  # File name to store history
        self.message_count = 0
        self.ensure_file_exists()
        self.reset_file()

    def reset_file(self):
        if os.path.exists(self.file_name):
            os.remove(self.file_name)

    def ensure_file_exists(self):
        """Ensure the history file exists, creating it if necessary."""
        # Check if the file already exists
        print("ensuring file exists")
        if not os.path.exists(self.file_name):
            # If it doesn't exist, create an empty file with an empty structure
            with open(self.file_name, 'w') as file:
                json.dump({}, file)

    def update_history(self, left_points, right_points, write_to_file=False):
        """Update the history with new left and right points, optionally writing to file."""
        # Add the new points to the history
        self.left_history.extend(left_points)
        self.right_history.extend(right_points)

        # Increment message count
        self.message_count += 1

        # Optionally, write to file
        if write_to_file:
            print("writing to file")
            self.save_to_file()

    def get_history(self):
        """Return the history of left and right cone points."""
        return self.left_history, self.right_history

    def save_to_file(self):
        """Save the current history to a JSON file."""
        # Create a dictionary with the new data
        data = {
            "left_cones": self.left_history,
            "right_cones": self.right_history
        }

        # Check if the file exists
        if os.path.exists(self.file_name):
            # If file exists, load it and update data
            print("file exists and appending data")
            with open(self.file_name, "r") as file:
                existing_data = json.load(file)

            # Append new data to the existing file
            existing_data[f"message_{self.message_count}"] = data
        else:
            # If file does not exist, initialize the data
            print("file does not exist so we initialize the data")
            existing_data = {f"message_{self.message_count}": data}

        # Write the updated data to the file
        with open(self.file_name, "w") as file:
            json.dump(existing_data, file, indent=4)

    def load_from_file(self):
        """Load history from a JSON file."""
        if os.path.exists(self.file_name):
            with open(self.file_name, "r") as file:
                return json.load(file)
        else:
            print(f"File {self.file_name} not found.")
            return None
