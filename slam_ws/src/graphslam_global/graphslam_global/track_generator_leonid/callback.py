import matplotlib.pyplot as plt
import numpy as np
import csv

import settings
import track_generator as tg


def on_click(event):
    # If the track is generated, disconnect event listener
    if settings.TRACK_GENERATED:
        settings.FIG.canvas.mpl_disconnect(settings.MOUSE)
        return

    # Add clicked point to list
    settings.CLICKED_POINTS.append(np.array([event.xdata, event.ydata]))

    # Plot the point and annotate
    settings.NUM_TRACK_POINTS += 1
    track_point_plot = plt.plot(event.xdata, event.ydata, '.r')[0]
    annotation = plt.annotate(settings.NUM_TRACK_POINTS, [
                              event.xdata, event.ydata])
    settings.TRACK_POINT_PLOTS.append(track_point_plot)
    settings.ANNOTATIONS.append(annotation)

    # Update status
    settings.FIG.canvas.draw()
    print(f'Point {settings.NUM_TRACK_POINTS} added!')


def on_t_press(event):
    # If not is t-key, exit
    if event.key != 't':
        return

    # If track is generated, disconnect event listener
    if settings.TRACK_GENERATED:
        settings.FIG.canvas.mpl_disconnect(settings.T_KEY)
        return

    # Generate track
    center_line, right_line, left_line, settings.YELLOW_CONES, \
        settings.BLUE_CONES, settings.ORANGE_CONES = tg.create_track(
            np.array(settings.CLICKED_POINTS))
    # So we can take the center line for testing purposes
    settings.CENTER_LINE = center_line

    # Plot centerline and border lines
    settings.CENTER_LINE_PLOT = plt.plot(
        center_line[:, 0], center_line[:, 1], '-k')[0]
    plt.plot(right_line[:, 0], right_line[:, 1], '--y', alpha=.5)
    plt.plot(left_line[:, 0], left_line[:, 1], '--b', alpha=.5)

    # Plot the cones
    plt.plot(settings.BLUE_CONES[:, 0], settings.BLUE_CONES[:, 1], '*b')
    plt.plot(settings.YELLOW_CONES[:, 0], settings.YELLOW_CONES[:, 1], '*y')
    plt.plot(settings.ORANGE_CONES[:, 0], settings.ORANGE_CONES[:, 1],
             linestyle='None', marker='*', color='tab:orange')

    # Update status
    plt.title('Track generated. Press \'h\' to toggle centerline')
    settings.FIG.canvas.draw()
    settings.TRACK_GENERATED = True
    print('Track generated.')


def on_h_press(event):
    # If wrong key and we have no track yet, exit
    if event.key != 'h' or not settings.TRACK_GENERATED:
        return

    # Toggle centerline visiblility
    is_invisible = settings.CENTER_LINE_PLOT.get_visible()
    settings.CENTER_LINE_PLOT.set_visible(not is_invisible)

    # Toggle track point and annotation visibility
    for track_point, annotation in zip(settings.TRACK_POINT_PLOTS, settings.ANNOTATIONS):
        annotation.set_visible(not is_invisible)
        track_point.set_visible(not is_invisible)

    # Update status
    settings.FIG.canvas.draw()
    on_off = 'off' if is_invisible else 'on'
    print(f'Centerline toggled {on_off}')


def on_x_press(event):
    # Exit if wrong key press or if we have generated a track or if we have no points
    if event.key != 'x' or settings.TRACK_GENERATED or settings.NUM_TRACK_POINTS == 0:
        return

    # Remove and decrement counter
    settings.CLICKED_POINTS.pop()
    settings.NUM_TRACK_POINTS -= 1
    track_point_plot = settings.TRACK_POINT_PLOTS.pop()
    annotation = settings.ANNOTATIONS.pop()
    track_point_plot.remove()
    annotation.remove()

    # Update status
    settings.FIG.canvas.draw()
    print(f'Point {settings.NUM_TRACK_POINTS + 1} removed')


def on_d_press(event):
    # If wrong key or track has been generated or already saved, exit
    if event.key != 'd' or not settings.TRACK_GENERATED or settings.TRACK_SAVED:
        return

    # np.arrays of color chars to append to coordinates
    blue_text = np.full(fill_value='b', shape=(
        len(settings.BLUE_CONES), 1), dtype=str)
    yellow_text = np.full(fill_value='y', shape=(
        len(settings.YELLOW_CONES), 1), dtype=str)
    orange_text = np.full(fill_value='o', shape=(
        len(settings.ORANGE_CONES), 1), dtype=str)

    # Append colors to coordinate arrays
    blue_with_field = np.hstack([settings.BLUE_CONES, blue_text])
    yellow_with_field = np.hstack([settings.YELLOW_CONES, yellow_text])
    orange_with_field = np.hstack([settings.ORANGE_CONES, orange_text])

    # Write .csv file
    with open(f'./generated_files/track.csv', 'w', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow(['x', 'y', 'color'])
        writer.writerows(orange_with_field)
        writer.writerows(blue_with_field)
        writer.writerows(yellow_with_field)
        file.close()

    # Update status
    settings.TRACK_SAVED = True
    plt.title('Track saved as .csv file!')
    settings.FIG.canvas.draw()
    print('Track saved!')
