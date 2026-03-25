import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from numpy import array
from nav2_msgs.action import NavigateToPose
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
# ________________________________________________________________________________


class robot:
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, node, name):
        self.node = node
        self.assigned_point = []
        self.name = name
        self.global_frame = node.get_parameter_or('global_frame', '/map').value
        self.robot_frame = node.get_parameter_or('robot_frame', 'base_link').value
        self.plan_service = node.get_parameter_or(
            'plan_service', '/plan').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        # Wait for transform
        self.node.get_logger().info('Waiting for the robot transform')
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.name+'/'+self.robot_frame,
                    rclpy.time.Time())
                break
            except Exception as e:
                rclpy.spin_once(self.node, timeout_sec=0.1)

        self.position = array([trans.transform.translation.x, trans.transform.translation.y])
        self.assigned_point = self.position

        self.client = ActionClient(self.node, NavigateToPose, self.name+'/navigate_to_pose')
        self.client.wait_for_server()

        self.make_plan_client = self.node.create_client(GetPlan, self.name+self.plan_service)
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame

    def getPosition(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.name+'/'+self.robot_frame,
                rclpy.time.Time())
            self.position = array([trans.transform.translation.x, trans.transform.translation.y])
        except Exception as e:
            self.node.get_logger().warn(f'Transform lookup failed: {e}')
        return self.position

    def sendGoal(self, point):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(point[0])
        goal_msg.pose.pose.position.y = float(point[1])
        goal_msg.pose.pose.orientation.w = 1.0
        self.client.send_goal_async(goal_msg)
        self.assigned_point = array(point)

    def cancelGoal(self):
        self.client._cancel_goal_async()
        self.assigned_point = self.getPosition()
# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    index = int((floor((Xp[1]-Xstarty)/resolution) * width) +
                (floor((Xp[0]-Xstartx)/resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i/mapData.info.width)*mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
    return array([x, y])
# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
                    infoGain += 1
    return infoGain*(mapData.info.resolution**2)
# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if(mapData.data[i] == -1 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                        infoGain[j] -= 1
    return infoGain
# ________________________________________________________________________________


def pathCost(path):
    if (len(path) > 0):
        i = len(path)//2
        p1 = array([path[i-1].pose.position.x, path[i-1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1-p2)*(len(path)-1)
    else:
        return inf
# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == 1):
                    return True
    return False
# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    result = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :]-x)
        if (n1 < n):
            n = n1
            result = i
    return result
# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i]-x)
        if (n1 < n):
            n = n1
            result = i
    return result
# ________________________________________________________________________________


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))
    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100


