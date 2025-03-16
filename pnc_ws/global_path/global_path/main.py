import rclpy
from rclpy.node import Node
import numpy as np
from feb_msgs.msg import State
from feb_msgs.msg import FebPath
from feb_msgs.msg import Map
from std_msgs.msg import Bool
#from .global_opt_settings import GlobalOptSettings as settings
from all_settings.all_settings import GlobalOptSettings as settings
from all_settings.all_settings import MPCSettings as mpc_settings
from .global_opt_compiled import CompiledGlobalOpt
from .ConeOrdering import ConeOrdering, N_point_generator
from eufs_msgs.msg import ConeArrayWithCovariance
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
import time

class GlobalPath(Node):
    def __init__(self):
        super().__init__("global_path")

        #Publishers
        self.pc_publisher = self.create_publisher(FebPath, '/path/global', 1)
        self.cp_publisher = self.create_publisher(Bool, '/path/finished', 1)
        self.viz_publisher = self.create_publisher(PointCloud, '/path/global/viz', 1)
        #Subscribers
        self.pc_subscriber = self.create_subscription(Map, '/slam/map/global', self.listener_cb, 1)
        # self.pc_subscriber = self.create_subscription(ConeArrayWithCovariance, '/ground_truth/track', self.listener_cb, 1)

        self.g = CompiledGlobalOpt(**settings)
        self.g.construct_solver()

    def listener_cb(self, msg: ConeArrayWithCovariance):
        left = [(6.11, 2.8), (6.109119415283203, 2.77589750289917), (8.934219360351562, 3.0638973712921143), (12.868500709533691, 3.012697458267212), (16.50389862060547, 3.4246973991394043), (20.136499404907227, 4.427197456359863), (22.62929916381836, 5.538097381591797), (25.49173927307129, 6.740597248077393), (28.138639450073242, 6.202697277069092), (30.521839141845703, 5.081397533416748), (32.50313949584961, 3.3362975120544434), (33.89194107055664, 1.401397466659546), (34.47583770751953, -1.504062533378601), (33.998939514160156, -5.023182392120361), (32.99224090576172, -8.059272766113281), (32.09803771972656, -10.299802780151367), (30.182140350341797, -13.539742469787598), (27.282838821411133, -15.564172744750977), (24.113739013671875, -17.043882369995117), (21.25337028503418, -18.83869171142578), (18.06159019470215, -20.45490264892578), (14.420599937438965, -22.263202667236328), (10.502239227294922, -24.330303192138672), (7.251099586486816, -26.4215030670166), (3.651329517364502, -27.45490264892578), (0.7883395552635193, -26.945703506469727), (-1.4627604484558105, -25.224702835083008), (-3.2429604530334473, -23.575801849365234), (-5.143360614776611, -21.38580322265625), (-5.6176605224609375, -17.860652923583984), (-5.938460350036621, -15.4548921585083), (-5.556060314178467, -12.859972953796387), (-5.120860576629639, -10.299802780151367), (-4.784360408782959, -7.257342338562012), (-4.843460559844971, -4.028892517089844), (-3.7221603393554688, -0.7304625511169434), (-2.1866605281829834, 1.4138973951339722), (0.24443955719470978, 2.190197467803955)]
        right = [(6.089759349822998, -1.8552625179290771), (6.089759349822998, -1.8552625179290771), (8.982789993286133, -1.796582579612732), (12.939980506896973, -1.9238924980163574), (16.51483917236328, -1.6030025482177734), (20.14760971069336, -0.3732425570487976), (23.430938720703125, 0.49919745326042175), (25.965240478515625, 1.5015974044799805), (25.965240478515625, 1.5015974044799805), (27.96493911743164, 0.9834974408149719), (27.96493911743164, 0.9834974408149719), (29.605138778686523, -1.1794525384902954), (29.605138778686523, -1.1794525384902954), (29.706039428710938, -4.192082405090332), (28.87574005126953, -6.820742607116699), (28.10573959350586, -8.797102928161621), (26.676239013671875, -10.299802780151367), (25.072938919067383, -11.731022834777832), (22.419269561767578, -13.14719295501709), (20.26255989074707, -14.240602493286133), (14.787479400634766, -16.96703338623047), (11.020049095153809, -18.86309242248535), (7.818509578704834, -20.855302810668945), (5.429309368133545, -22.412302017211914), (3.2358596324920654, -23.007902145385742), (3.2358596324920654, -23.007902145385742), (0.9659395813941956, -21.971603393554688), (0.9659395813941956, -21.971603393554688), (-0.9300604462623596, -19.282712936401367), (-1.157760500907898, -16.07309341430664), (-1.157760500907898, -16.07309341430664), (-1.0045604705810547, -12.92978286743164), (-0.6089604496955872, -10.299802780151367), (-0.3480604290962219, -7.242682456970215), (-0.34576043486595154, -4.511722564697266), (0.7813395857810974, -2.6917724609375), (0.7813395857810974, -2.6917724609375), (0.7813395857810974, -2.6917724609375)]
        # left = np.array([list(i) for i in left])
        # right = np.array([list(i) for i in right])
        # left = [[cone.point.x, cone.point.y] for cone in msg.blue_cones]
        # right = [[cone.point.x, cone.point.y] for cone in msg.yellow_cones]
        # left.sort(key=lambda x: (x[0]-5)**2 + x[1]**2)
        # right.sort(key=lambda x: (x[0]-5)**2 + x[1]**2)
        # p = PairPath(left, right, 100)
        # left = np.vstack([p.l[-20:], p.l[:-20]])
        # right = np.vstack([p.r[-20:], p.r[:-20]])
        # ConeOrdering(msg)
        
        res = self.g.solve(np.array(left), np.array(right))
        states, controls = self.g.to_constant_tgrid(mpc_settings.DT*0.1, **res)
        print('here!!!')
        print(states.shape, controls.shape)
        
        msg = FebPath()
        msg.x = states[:, 0].flatten().tolist()
        msg.y = states[:, 1].flatten().tolist()
        msg.psi = states[:, 2].flatten().tolist()
        msg.v = states[:, 3].flatten().tolist()
        msg.th = states[:, 4].flatten().tolist()

        msg.a = controls[:, 0].flatten().tolist()
        msg.thdot = controls[:, 1].flatten().tolist()
        
        self.pc_publisher.publish(msg)

        msg = Bool()
        msg.data = True
        self.cp_publisher.publish(msg)


        pc_msg = PointCloud()
        pts = []
        for x in states:
            pts.append(Point32())
            pts[-1].x = x[0]
            pts[-1].y = x[1]
            pts[-1].z = 0.0
        pc_msg.points = pts
        pc_msg.header.frame_id = "map"
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_publisher.publish(pc_msg)
        # we don't need this anymore and the expression graph isn't exactly small, so let's free stuff
        self.destroy_node()

def main(args=None):
    try:
        rclpy.init(args=args)
    except:
        print("global init failed")
    global_path_node = GlobalPath()
    try:
        rclpy.spin(global_path_node)
    except Exception as e:
        print("global path node stopped")
        print("finn and jake")
        raise e
    rclpy.shutdown()

if __name__ == "__main__":
    main()
