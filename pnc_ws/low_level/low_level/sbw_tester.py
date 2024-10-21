import rclpy
from std_msgs.msg import Float64

def main(args=None):
    rclpy.init(args=args)
    test_sender = rclpy.create_node("sbw_tester")
    pub = test_sender.create_publisher(Float64, "/control/steer", 1)

    while True:
        try: (msg := Float64()).data = float(input("/control/steer: "))
        except ValueError: continue
        pub.publish(msg)

if __name__ == '__main__': main()