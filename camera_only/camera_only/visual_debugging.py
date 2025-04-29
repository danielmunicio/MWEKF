import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

def publish_perception_visual(self, x_coordinates, y_coordinates, chosen_classes):
    cones_guess = PointCloud()
    positions = []
    print("X COORDS: ", x_coordinates)
    print("Y COORDS: ", y_coordinates)
    print("COLORS: ", chosen_classes)
    for x, y, color_value in zip(x_coordinates, y_coordinates, chosen_classes):
        print("X: ", x)
        print("Y: ", y)
        print("CLASS: ", color_value)
        positions.append(Point32())
        positions[-1].x = x
        positions[-1].y = y
        positions[-1].z = 0.0

    cones_guess.points = positions
    cones_guess.header.frame_id= "map"
    cones_guess.header.stamp = self.get_clock().now().to_msg()

    self.perception_visual_pub.publish(cones_guess)
