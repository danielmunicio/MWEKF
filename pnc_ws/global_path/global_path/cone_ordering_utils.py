def convert_coordinates_to_normal(points):
    new_points = []
    for point in points:
        p1, p2 = point
        new_points.append((p1, -p2))
    return new_points

