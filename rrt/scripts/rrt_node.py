#!/usr/bin/python3

"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
import cv2

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
from visualization_msgs.msg import Marker, MarkerArray

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
        goal_marker_topic = "/goal_marker"
        node_arr_topic = "/rrt_node_arr"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            #PoseStamped,
            Odometry,
            pose_topic,
            self.pose_callback,
            1)

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)

        # Some paramters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_res', 0.05),
                ('grid_w',5),
                ("grid_h", 2),
                ("num_pts", 100),
                ('step_size', 0.25),
                ('goal_thresh', 0.05),
                ('max_speed', 2.0),
                ('L', 3.0)
            ]
        )

        # Set paramters from config file
        self.map_res      = self.get_parameter("map_res").value # Dimensions in meters of occ griid
        self.grid_w       = self.get_parameter("grid_w").value # Width in meters of grid
        self.grid_h       = self.get_parameter("grid_h").value # Height in meters of grid
        self.N            = self.get_parameter("num_pts").value # Number of points to sample for the tree
        self.step_size    = self.get_parameter("step_size").value
        self.goal_thresh  = self.get_parameter("goal_thresh").value
        self.m_max_speed  = self.get_parameter("max_speed").value
        self.m_L          = self.get_parameter("L").value
        self.m_marker_idx = 0
        self.x_goal       = None

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

        # Visualization publishers
        self.goal_node_pub = self.create_publisher(
            Marker,
            goal_marker_topic,
            1
        )

        self.marker_arr_pub = self.create_publisher(
            MarkerArray,
            node_arr_topic,
            1
        )

        self.path_pub = self.create_publisher(
            MarkerArray,
            "/selected_path",
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
        occ_grid_msg.header.frame_id = "/ego_racecar/laser"
        print(f"Number of Points: {self.N}")
        # Create an instaneous occupancy gird
        self.inst_occ_grid = np.ones((int(self.grid_h/self.map_res), int(self.grid_w/self.map_res)))

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
            # Offset y by half the width because bottom left of map is to the left of car
            y_m = rngs[i] * np.sin(ang)
            y_pix = self.grid_w/(2*self.map_res) + y_m/self.map_res

            # Mark free space as the valid pixels
            msk = (ray_pts_x <= x_pix) & (ray_pts_x>= 0) & (ray_pts_x < self.grid_h/self.map_res) & \
                   (ray_pts_y >= 0) & (ray_pts_y < self.grid_w/self.map_res)
            msk_ray_pts_x = ray_pts_x[msk].astype("int")
            msk_ray_pts_y = ray_pts_y[msk].astype("int")

            # Since we are doing an instaneous grid, we can just mark the hits as occupied
            # NOTE: If we do a temporal one, we can just update the temporal using the instanteous
            self.inst_occ_grid[msk_ray_pts_x, msk_ray_pts_y] = 0.0

        # Dilate the grid
        # self.inst_occ_grid = 1 - cv2.dilate(1 - self.inst_occ_grid, (3,3))

        # Fill in the data and publish the occ grid
        data = (np.flip(self.inst_occ_grid,1)*(100)).astype("int8").flatten(order="C").tolist()
        occ_grid_msg.data = data
        self.occ_grid_pub.publish(occ_grid_msg)

        # import cv2
        # Flip to the right orientation.
        # Image to ego car
        #self.inst_occ_grid = np.flip(self.inst_occ_grid*255, axis = (0,1))
        # cv2.imshow("OpenCV Test", self.inst_occ_grid)
        # cv2.waitKey()

    def get_ego_coords_from_grid(self, x_pix, y_pix):
        # Offset the middle of the grid
        x_m = (x_pix * self.map_res)
        y_m = y_pix * self.map_res - (self.grid_w/2)
        return x_m, y_m

    def find_goal_node(self):
        # Deepest point that is in the middle
        free_space_x, free_space_y = np.where(self.inst_occ_grid<0.5)
        occ_space_x, occ_space_y = np.where(self.inst_occ_grid<0.5)
        x_goal = np.max(free_space_x)

        # Find left wall and find right wall
        occ_ys = self.inst_occ_grid[x_goal]
        grad_y = np.gradient(occ_ys)
        y_low = np.argmin(grad_y)
        y_hi = np.argmax(grad_y)

        y_goal = y_low+y_hi/2
        goal_node = RRTNode()
        goal_node.x = x_goal
        bounded_y = y_goal
        goal_node.y = bounded_y
                          
        # Get coordinates in meters for marker
        x_m, y_m = self.get_ego_coords_from_grid(goal_node.x, goal_node.y)

        # Visualize the goal node, need to go from image to ego frame
        marker = Marker();
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = self.m_marker_idx
        #self.m_marker_idx += 1
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "ego_racecar/base_link"
        marker.pose.position.x = x_m
        marker.pose.position.y = y_m
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.goal_node_pub.publish(marker)

        return goal_node

    def create_marker_arr_from_tree(self, tree):
        # Create a MarkerArray
        marker_array = MarkerArray()
        i = 0
        for i, node in enumerate(tree):
            # Get coordinates in meters for marker
            x_m, y_m = self.get_ego_coords_from_grid(node.x, node.y)

            # Points
            # Visualize the goal node, need to go from image to ego frame
            marker = Marker();
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i
            i+=1
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "ego_racecar/base_link"
            marker.pose.position.x = x_m
            marker.pose.position.y = y_m
            marker.pose.position.z = 0.25
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            if node.parent is not None:
                parent_x, parent_y = self.get_ego_coords_from_grid(node.parent.x, node.parent.y)

                # LINE STRIPS
                line_marker = Marker()
                line_marker.header.frame_id = "ego_racecar/base_link"
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.id = len(tree) + i
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                point1 = Point()
                point1.x=x_m
                point1.y=y_m
                point2 = Point()
                point2.x = parent_x
                point2.y = parent_y
                line_marker.points.append(point1)
                line_marker.points.append(point2)
                line_marker.scale.x = 0.02  # Line width
                line_marker.color.r = 0.0
                line_marker.color.g = 0.0
                line_marker.color.b = 1.0
                line_marker.color.a = 1.0
                marker_array.markers.append(line_marker)
        return marker_array

    def visualize_path(self, path):
        # Create a MarkerArray
        marker_array = MarkerArray()
        for i in range(path.shape[0] -1):
            p1 = path[i]
            p2 = path[i+1]

            # LINE STRIPS
            line_marker = Marker()
            line_marker.header.frame_id = "ego_racecar/base_link"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            point1 = Point()
            point1.x=path[i, 0]
            point1.y=path[i, 1]
            point2 = Point()
            point2.x = path[i+1,0]
            point2.y = path[i+1, 1]
            line_marker.points.append(point1)
            line_marker.points.append(point2)
            line_marker.scale.x = 0.08  # Line width
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.lifetime.sec = 1
            marker_array.markers.append(line_marker)
        self.path_pub.publish(marker_array)

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
    
        ### Acutal Loop ####
        # 1. Sample in free space
        xs, ys = self.sample()

        # Visualize randomly sampled points
        # test = []
        # for x,y in zip(xs, ys):
        #     node = RRTNode()
        #     node.x = x
        #     node.y = y
        #     test.append(node)
        # marker_arr = self.create_marker_arr_from_tree(test)

        # Start to create the tree
        tree = []
        start_pt = RRTNode()
        start_pt.x = xs[0]
        start_pt.y = ys[0]
        start_pt.is_root = True
        tree.append(start_pt)

        # Find a goal node that is near the edge of free space
        self.x_goal = self.find_goal_node()

        for i in range(1, len(xs)-1, 1):
            # 2. Get the sampled point
            x_rand = np.array([xs[i], ys[i]])

            # 3. Get the nearest point to the sample point
            nearest_idx, x_nearest = self.nearest(tree, x_rand)

            # 4. Get the new point, steering towards a collision free path
            x_new = self.steer(x_nearest, x_rand)

            # 5. If collision free, add to the graph
            if x_new is not None:
                x_new.parent = tree[nearest_idx]
                tree.append(x_new)
                dist_to_goal = np.linalg.norm(np.array([x_new.x, x_new.y]) - np.array([self.x_goal.x, self.x_goal.y]))
                if dist_to_goal < self.goal_thresh:
                    break
        
        # Now we have the tree, first thing we can do is visualize the points
        marker_arr = self.create_marker_arr_from_tree(tree)
        self.marker_arr_pub.publish(marker_arr)

        # Find the path and visualize it
        path = np.array(self.find_path(tree, tree[-1]))
        goal_dist = np.linalg.norm(path - np.array([self.x_goal.x, self.x_goal.y]).reshape(1,2), axis=1)

        # If this is positive, discard
        path_forward = path[1:]
        relative_dists = np.array([goal_dist[i] - goal_dist[i-1] for i in range(1, len(path))])
        new_path = np.vstack((path[0], path_forward[relative_dists < 0], np.array([self.x_goal.x, self.x_goal.y])))
        new_path_ego = np.array([self.get_ego_coords_from_grid(node[0], node[1]) for node in new_path])
        self.visualize_path(new_path_ego)

        # Use pure pursuit for to drive the car
        follow_idx = self.find_best_waypt(new_path_ego)
        steering_angle = self.calc_steering_angle(new_path_ego[follow_idx])

        # Publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()

        # Set speed
        speed = 0.0;
        steer_angle_deg = abs(steering_angle) * 180/math.pi;
        #print(f"Steer angle: {steer_angle_deg}")
  
        if steer_angle_deg >= 0 and steer_angle_deg < 10:
            speed = self.m_max_speed;
            self.m_L = self.m_max_speed * 0.50;
        elif steer_angle_deg >= 10 and steer_angle_deg < 20:
            speed = self.m_max_speed/2;
            self.m_L = self.m_max_speed * 0.25;
        else:
            speed = self.m_max_speed/4;
            self.m_L = self.m_max_speed * 0.25;

        # Set steering angle and speed
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


    def find_best_waypt(self, waypts):
        dist_from_tgts = []
        euclid_dists = np.linalg.norm(waypts, axis = 1)
        dist_from_tgts = np.abs(euclid_dists - self.m_L)
        min_idx = np.argmin(dist_from_tgts)
        return min_idx

    def calc_steering_angle(self, waypt):
        gamma = 2 * waypt[1] / self.m_L**2;
        return gamma;

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        idxs = np.where(self.inst_occ_grid < 0.5)
        pts = np.vstack(idxs)
        num_pts = min(pts.shape[1], self.N)
        chosen_idxs = np.random.choice(pts.shape[1], size=num_pts, replace=False)
        x, y = pts[:, chosen_idxs]
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
        dist = [np.linalg.norm(np.array([node.x, node.y])-np.array(sampled_point)) for node in tree]
        return np.argmin(dist), tree[np.argmin(dist)]

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
        # Find the direction and the unit vector
        direction = np.array(sampled_point) - np.array([nearest_node.x, nearest_node.y])
        norm_direction = direction / np.linalg.norm(direction)

        # Limit the new point to be in the direction of the point
        new_xy = sampled_point +  self.step_size * norm_direction

        # Do a simple ray cast to see if this is a collision
        cast_x = nearest_node.x
        cast_y = nearest_node.y
        while cast_x < new_xy[0] and cast_y < new_xy[1]:
            # Return none if we hit an obstacle
            if self.inst_occ_grid[int(cast_x), int(cast_y)] > 0.5:
                return None
            
            # Continue down the ray
            cast_x += norm_direction[0]
            cast_y += norm_direction[1]

        # For the success case, return a node
        new_node = RRTNode()
        new_node.x = new_xy[0]
        new_node.y = new_xy[1]
        return new_node

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
        # Start with the goal node then backtrack
        path = []
        node = tree[-1]
        while node is not None:
            px = node.x
            py = node.y
            path.append((px, py))
            node = node.parent
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
