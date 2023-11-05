"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class RRTNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('RRT')

        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"
        drive_topic = "/drive"
        occ_grid_topic = "/occ_grid"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            #PoseStamped,
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)


        # Some paramters
        # TODO: Put these in a config file?
        self.map_res = 0.05 # meters per pixel
        self.grid_w = 2 # meters
        self.grid_h = 5 # meters

        # publishers
        # Ceate a drive message publisher, and other publishers that you might need
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            1
        )

        # Create occupancy grid publisher
        self.occ_grid_pub = self.create_publisher(
            OccupancyGrid,
            occ_grid_topic,
            1
        )

        # Other Params
        self.inst_occ_grid = None

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        # Occupancy grid message
        occ_grid_msg = OccupancyGrid()
        occ_grid_msg.info.width = int(self.grid_w/self.map_res)
        occ_grid_msg.info.height = int(self.grid_h/self.map_res)
        occ_grid_msg.info.resolution = self.map_res
        occ_grid_msg.info.origin.position.x = 0.0
        occ_grid_msg.info.origin.position.y = float(self.grid_w)/2
        occ_grid_msg.info.origin.orientation.w = 0.7071068 
        occ_grid_msg.info.origin.orientation.z = -0.7071068
        occ_grid_msg.header.frame_id = "/ego_racecar/base_link"

        # Create an instaneous occupancy gird
        self.inst_occ_grid = np.zeros((int(self.grid_h/self.map_res), int(self.grid_w/self.map_res)))

        # Loop over the lasers
        rngs = scan_msg.ranges
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

        # Pre compute the points in the grid for raycasting
        x_vals = np.arange(0, self.grid_h, self.map_res)

        for i, ang in enumerate(angles):
            # Transform the points to be on the ray
            ray_pts_x = x_vals*np.cos(ang) / self.map_res
            ray_pts_y = (x_vals*np.sin(ang) / self.map_res) + self.grid_w/(2*self.map_res)

            # Calculate end point
            x_m = rngs[i] * np.cos(ang)
            x_pix = x_m/self.map_res
            # # Offset y by half the width because bottom left of map is to the left of car
            y_m = rngs[i] * np.sin(ang)
            y_pix = self.grid_w/(2*self.map_res) + y_m/self.map_res
            # y_pix = max(y_pix, 0)
            # y_pix = min(y_pix, self.grid_w-1)


            # self.inst_occ_grid[int(x_pix), int(y_pix)] = 1.0

            # Find the points at or beyond the endpoint and make them occupied
            msk = (ray_pts_x >= x_pix) & (ray_pts_x>=0) & (ray_pts_x < self.grid_h/self.map_res) & \
                   (ray_pts_y >= 0) & (ray_pts_y < self.grid_w/self.map_res)
            msk_ray_pts_x = ray_pts_x[msk].astype("int")
            msk_ray_pts_y = ray_pts_y[msk].astype("int")

            # Since we are doing an instaneous grid, we can just mark the hits as occupied
            # NOTE: If we do a temporal one, we can just update the temporal using the instanteous
            self.inst_occ_grid[msk_ray_pts_x, msk_ray_pts_y] = 1.0

        # Fill in the data and publish the occ grid
        import cv2
        flipped_grid = np.flip(self.inst_occ_grid*255, axis = (0, 1))
        
        cv2.imshow("Yas", flipped_grid)
        cv2.waitKey()
        flipped_grid = np.flip(self.inst_occ_grid*255, axis = (0, 1))
        data = (self.inst_occ_grid*100).astype("int8").flatten(order="C").tolist()
        occ_grid_msg.data = data
        
        self.occ_grid_pub.publish(occ_grid_msg)

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # Early exit
        if self.inst_occ_grid is None:
            return None
        


        return None

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = None
        y = None
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = None
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
