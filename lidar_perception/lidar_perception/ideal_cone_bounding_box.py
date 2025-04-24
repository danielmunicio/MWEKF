import numpy as np
import matplotlib.pyplot as plt

def load_bounding_boxes(filename):
    # Load the CSV file using numpy
    try:
        data = np.loadtxt(filename, delimiter=',', skiprows=1)  # Skip the header
    except Exception as e:
        print(f"Error loading file: {e}")
        return None
    return data

def plot_bounding_box_distribution(data):
    if data is None:
        print("No data to plot.")
        return

    # Extract x_size and y_size from the loaded data
    x_size = data[:, 0]
    y_size = data[:, 1]

    # Create a figure and axis for plotting
    fig, ax = plt.subplots(1, 2, figsize=(12, 6))

    # Plotting the box plot (cat and whiskers plot)
    ax[0].boxplot([x_size, y_size], labels=['x_size', 'y_size'])
    ax[0].set_title("Box Plot of Bounding Box Sizes")
    ax[0].set_ylabel("Size")
    
    # Plotting the histograms for x_size and y_size
    ax[1].hist(x_size, bins=30, alpha=0.7, label='x_size', color='blue', edgecolor='black')
    ax[1].hist(y_size, bins=30, alpha=0.7, label='y_size', color='red', edgecolor='black')
    ax[1].set_title("Histogram of Bounding Box Sizes")
    ax[1].set_xlabel("Size")
    ax[1].set_ylabel("Frequency")
    ax[1].legend()

    # Show the plots
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Load the data from the CSV
    filename = 'cone_bounding_boxes.csv'
    data = load_bounding_boxes(filename)

    # Plot the distribution of bounding box sizes
    plot_bounding_box_distribution(data)
