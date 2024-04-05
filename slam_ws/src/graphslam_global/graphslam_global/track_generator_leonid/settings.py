import numpy as np
import matplotlib.pyplot as plt


def init():
    global TRACK_POINTS, TRACK_WIDTH, CLICKED_POINTS, NUM_TRACK_POINTS, \
        TRACK_GENERATED, TRACK_STEP, NUM_LINE_POINTS, CENTER_LINE_PLOT, \
        TRACK_POINT_PLOTS, ANNOTATIONS, FIG, MOUSE, T_KEY, H_KEY, X_KEY, \
        D_KEY, ORANGE_CONES, BLUE_CONES, YELLOW_CONES, TRACK_SAVED, CENTER_LINE

    FIG = plt.figure('FEB Track generator')
    TRACK_GENERATED = False
    NUM_TRACK_POINTS = 0
    NUM_LINE_POINTS = 1000
    TRACK_STEP = NUM_LINE_POINTS // 100
    TRACK_WIDTH = 6.
    CLICKED_POINTS = []
    CENTER_LINE_PLOT = None
    TRACK_POINT_PLOTS = []
    ANNOTATIONS = []
    MOUSE, T_KEY, H_KEY, X_KEY, D_KEY = int, int, int, int, int
    ORANGE_CONES, BLUE_CONES, YELLOW_CONES = np.array, np.array, np.array
    TRACK_SAVED = False
    CENTER_LINE = np.array

    TRACK_POINTS = np.array([
        [0., 0.],
        [50., 0.],
        [50., 60.],
        [-100., 60.],
        [-100., -50.],
        [-50., -50.],
        [-50., 0.]
    ])

    # TRACK_POINTS = np.array([
    #     [6.55525,  3.05472],
    #     [6.17284,  2.802609],
    #     [5.53946,  2.649209],
    #     [4.93053,  2.444444],
    #     [4.32544,  2.318749],
    #     [3.90982,  2.2875],
    #     [3.51294,  2.221875],
    #     [3.09107,  2.29375],
    #     [2.64013,  2.4375],
    #     [2.275444,  2.653124],
    #     [2.137945,  3.26562],
    #     [2.15982,  3.84375],
    #     [2.20982,  4.31562],
    #     [2.334704,  4.87873],
    #     [2.314264,  5.5047],
    #     [2.311709,  5.9135],
    #     [2.29638,  6.42961],
    #     [2.619374,  6.75021],
    #     [3.32448,  6.66353],
    #     [3.31582,  5.68866],
    #     [3.35159,  5.17255],
    #     [3.48482,  4.73125],
    #     [3.70669,  4.51875],
    #     [4.23639,  4.58968],
    #     [4.39592,  4.94615],
    #     [4.33527,  5.33862],
    #     [3.95968,  5.61967],
    #     [3.56366,  5.73976],
    #     [3.78818,  6.55292],
    #     [4.27712,  6.8283],
    #     [4.89532,  6.78615],
    #     [5.35334,  6.72433],
    #     [5.71583,  6.54449],
    #     [6.13452,  6.46019],
    #     [6.54478,  6.26068],
    #     [6.7873,  5.74615],
    #     [6.64086,  5.25269],
    #     [6.45649,  4.86206],
    #     [6.41586,  4.46519],
    #     [5.44711,  4.26519],
    #     [5.04087,  4.10581],
    #     [4.70013,  3.67405],
    #     [4.83482,  3.4375],
    #     [5.34086,  3.43394],
    #     [5.76392,  3.55156],
    #     [6.37056,  3.8778],
    #     [6.53116,  3.47228]
    # ])
