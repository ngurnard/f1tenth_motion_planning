// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS params
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Resolution of the occupancy grid in meters";
    this->declare_parameter("resolution", 0.05, param_desc);
    param_desc.description = "Width of the occupancy grid in meters";
    this->declare_parameter("width_m", 2.0, param_desc);
    param_desc.description = "Height of the occupancy grid in meters";
    this->declare_parameter("height_m", 1.0, param_desc);

    // TODO: get the actual car width
    param_desc.description = "Width in the car in meters";
    this->declare_parameter("car_width", .15, param_desc);

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    // occupancy grid
    occupancy_grid.header.frame_id = "laser";
    occupancy_grid.info.resolution = this->get_parameter("resolution").as_double();
    int width_temp = this->get_parameter("width_m").as_double() / occupancy_grid.info.resolution;
    occupancy_grid.info.height = this->get_parameter("height_m").as_double() / occupancy_grid.info.resolution;
    // make sure occ cell is odd in width so can drive straight 
    if (width_temp % 2 == 0) {
        occupancy_grid.info.width = width_temp + 1;
    } else {
        occupancy_grid.info.width = width_temp;
    }
    // store the top corner distances (max possible)
    max_occ_dist = sqrt(pow(this->get_parameter("width_m").as_double(), 2)/4.0 + pow(this->get_parameter("height_m").as_double(), 2));

    
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    /*
     The scan callback, update your occupancy grid here
    Args:
       scan_msg (*LaserScan): pointer to the incoming scan message
    Returns:
    */

    // TODO: update your occupancy grid
    occupancy_grid.info.map_load_time = scan_msg->header.stamp;
    occupancy_grid.header.stamp = scan_msg->header.stamp;
    occupancy_grid.data = {0};

    int index_neg90 = (int) ((-90.0 - scan_msg->angle_min) / scan_msg->angle_increment);
    int index_pos90 = (int) ((90.0 - scan_msg->angle_min) / scan_msg->angle_increment);

    for(int i=index_neg90; i< index_pos90; i++) 
    {
        if(scan_msg->ranges[i] < max_occ_dist)
        {
            // forward and left to right on the grid
            int fwd = (int) (scan_msg->ranges[i] * cos(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution);
            int ltr = (int) (scan_msg->ranges[i] * sin(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution);
            // the origin of the grid is the bottom left so adjust from car frame calcs
            int x = -(ltr - occupancy_grid.info.width/2);
            int y = fwd;
            occupancy_grid.data[x*occupancy_grid.info.width + y] = 100;
            occupancy_grid.data.push_back(100);
        }
    }
    
    
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree;

    // TODO: fill in the RRT main loop



    // path found as Path message

}

std::vector<double> RRT::sample() {
    /*
    This method returns a sampled point from the free space
    You should restrict so that it only samples a small region
    of interest around the car's current position
    Args:
    Returns:
        sampled_point (std::vector<double>): the sampled point in free space

    TODO: fill in this method
    look up the documentation on how to use std::mt19937 devices with a distribution
    the generator and the distribution is created for you (check the header file)
    */
    
    std::mt19937 rand_gen(time(nullptr)); // make random number generator based on some seed time(nullptr)
    
    std::vector<double> sampled_point;
    sampled_point.push_back(rand_gen());
    sampled_point.push_back(rand_gen());

    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    /*
    This method returns the nearest node on the tree to the sampled point
    Args:
        tree (std::vector<RRT_Node>): the current RRT tree
        sampled_point (std::vector<double>): the sampled point in free space
    Returns:
        nearest_node (int): index of nearest node on the tree
    */

    int nearest_node = 0;
    double nearest_dist = MAXFLOAT;
    double x, y, dist;
    for(int i = 0; i < tree.size; i++)
    {
        x = tree[i].x - sampled_point[0];
        y = tree[i].y - sampled_point[1];
        dist = sqrt(pow(x,2) - pow(y,2));
        if(dist < nearest_dist)
        {
            nearest_node = i;
            nearest_dist = dist;
        }
    }

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    /* 
    The function steer:(x,y)->z returns a point such that z is “closer” 
    to y than x is. The point z returned by the function steer will be 
    such that z minimizes ||z−y|| while at the same time maintaining 
    ||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    basically, expand the tree towards the sample point (within a max dist)

    Args:
       nearest_node (RRT_Node): nearest node on the tree to the sampled point
       sampled_point (std::vector<double>): the sampled point in free space
    Returns:
       new_node (RRT_Node): new node created from steering
    */
    RRT_Node new_node;
    double x, y, dist, theta;
    
    x = sampled_point[0] - nearest_node.x;
    y = sampled_point[1] - nearest_node.y;
    dist = sqrt(pow(x,2) - pow(y,2));
        
    if (dist > max_expansion_dist) 
    {
        theta = atan2(y,x);
        new_node.x = nearest_node.x + max_expansion_dist * cos(theta);
        new_node.y = nearest_node.y + max_expansion_dist * sin(theta);
    }
    else
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    /* 
    This method returns a boolean indicating if the path between the 
    nearest node and the new node created from steering is collision free
    Args:
       nearest_node (RRT_Node): nearest node on the tree to the sampled point
       new_node (RRT_Node): new node created from steering
    Returns:
       collision (bool): true if in collision, false otherwise
    */

    bool collision = false;
    // TODO: fill in this method

    // use AABB (axis-aligned bounding box)
    // refer here: https://www.realtimerendering.com/intersections.html
    // this->occupancy_grid; // use this somehow


    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    /*
    This method checks if the latest node added to the tree is close
    enough (defined by goal_threshold) to the goal so we can terminate
    the search and find a path
    Args:
      latest_added_node (RRT_Node): latest addition to the tree
      goal_x (double): x coordinate of the current goal
      goal_y (double): y coordinate of the current goal
    Returns:
      close_enough (bool): true if node close enough to the goal
    */

    bool close_enough = false;
    double x, y, dist;
    
    x = latest_added_node.x - goal_x;
    y = latest_added_node.y - goal_y;
    dist = sqrt(pow(x,2) + pow(y,2));
    if(dist < goal_threshold)
    {
        close_enough = true;
    }

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    /*
    This method traverses the tree from the node that has been determined
    as goal
    Args:
      latest_added_node (RRT_Node): latest addition to the tree that has been
         determined to be close enough to the goal
    Returns:
      path (std::vector<RRT_Node>): the vector that represents the order of
         of the nodes traversed as the found path
    */
    
    std::vector<RRT_Node> found_path;
    RRT_Node curr_node;

    while (curr_node.parent != -1)
    {
        found_path.push_back(curr_node);
        curr_node = tree[curr_node.parent];
    }

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}