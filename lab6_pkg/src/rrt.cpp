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
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()), goal_y(0.0),
            pose_topic("ego_racecar/odom"), scan_topic("/scan"), cur_wpt_topic("/waypoint"),
            drive_topic("/drive"), occ_grid_topic("/occ_grid"), local_frame("/laser")
{
    // ROS publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
    occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occ_grid_topic, 1);

    // ROS params
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Resolution of the occupancy grid in meters";
    this->declare_parameter("resolution", 0.1, param_desc);
    param_desc.description = "Width of the occupancy grid in meters";
    this->declare_parameter("width_m", 2.0, param_desc);
    param_desc.description = "Height of the occupancy grid in meters";
    this->declare_parameter("height_m", 1.0, param_desc);
    param_desc.description = "Lookahead distance in meters";
    this->declare_parameter("L", 1.0, param_desc);
    param_desc.description = "Kp value";
    this->declare_parameter("Kp", 0.1);
    param_desc.description = "Velocity";
    this->declare_parameter("v", 2.0);

    // // TODO: get the actual car width
    // param_desc.description = "Width in the car in meters";
    // this->declare_parameter("car_width", .15, param_desc);

    this->goal_x = this->get_parameter("height_m").get_parameter_value().get<float>();
    goal_threshold = this->get_parameter("resolution").get_parameter_value().get<float>() / 2.0;
    max_expansion_dist = this->get_parameter("resolution").get_parameter_value().get<float>() / 10.0;

    // ROS subscribers
    // TODO: create subscribers as you need
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    waypoint_sub_ = this->create_subscription<interfaces_hot_wheels::msg::Waypoint>(
      cur_wpt_topic, 1, std::bind(&RRT::waypoint_callback, this, std::placeholders::_1));

    // random generator
    std::uniform_real_distribution<> x_temp(
        0.0,
        this->get_parameter("height_m").get_parameter_value().get<float>());
    std::uniform_real_distribution<> y_temp(
        -this->get_parameter("width_m").get_parameter_value().get<float>(),
         this->get_parameter("width_m").get_parameter_value().get<float>());
    x_dist.param(x_temp.param());
    y_dist.param(y_temp.param());

    // occupancy grid
    occupancy_grid.header.frame_id = "laser";
    occupancy_grid.info.resolution = this->get_parameter("resolution").get_parameter_value().get<float>();
    int width_temp = this->get_parameter("width_m").get_parameter_value().get<float>() / occupancy_grid.info.resolution;
    occupancy_grid.info.height = this->get_parameter("height_m").get_parameter_value().get<float>() / occupancy_grid.info.resolution;
    // make sure occ cell is odd in width so can drive straight
    if (width_temp % 2 == 0) {
        occupancy_grid.info.width = width_temp + 1;
    } else {
        occupancy_grid.info.width = width_temp;
    }
    // store the top corner distances (max possible)
    max_occ_dist = sqrt(pow(this->get_parameter("width_m").get_parameter_value().get<float>(), 2)/4.0 + pow(this->get_parameter("height_m").get_parameter_value().get<float>(), 2));


    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");

    // visualization code
    rrt_goal_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_goal", 1);
    rrt_node_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_nodes", 1);
    rrt_branch_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_branches", 1);
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    /*
     The scan callback, update your occupancy grid here
    Args:
       scan_msg (*LaserScan): pointer to the incoming scan message
    Returns:
    */

    // TODO: update your occupancy grid
    // cout << "scan callback" << endl;
    occupancy_grid.info.map_load_time = scan_msg->header.stamp;
    occupancy_grid.header.stamp = scan_msg->header.stamp;
    occupancy_grid.data = {0};

    float rad90 = 90.0 * M_PI / 180.0;
    int index_neg90 = (int) ((-rad90 - scan_msg->angle_min) / scan_msg->angle_increment);
    int index_pos90 = (int) ((rad90 - scan_msg->angle_min) / scan_msg->angle_increment);

    for(int i=index_neg90; i< index_pos90; i++)
    {  
        // cout << "i: " << i << endl;
        if(scan_msg->ranges[i] < max_occ_dist)
        {
            // forward and right to left on the grid
            int fwd = (int) (scan_msg->ranges[i] * cos(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution);
            int rtl = (int) (scan_msg->ranges[i] * sin(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution);
            // the origin of the grid is the bottom left so adjust from car frame calcs
            int x = -(rtl - occupancy_grid.info.width/2);
            int y = fwd;
            occupancy_grid.data[x*occupancy_grid.info.width + y] = 100;
        }
    }
    occ_grid_pub_->publish(occupancy_grid);
    // cout << "scan callback done" << endl;
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    /*
    The pose callback when subscribed to particle filter's inferred pose
    The RRT main loop happens here
    Args:
       pose_msg (*PoseStamped): pointer to the incoming pose message
    Returns:
       tree as std::vector
    */
    cout << "pose callback lol" << endl;
    
    tree.clear();
    
    root.x = 0;
    root.y = 0;
    root.cost = 0.0;
    root.parent = -1;
    root.is_root = true;
    root.index = 0;

    tree.push_back(root);
    RRT_Node steer_node;
    int count = 1;
    while (!is_goal(tree.back()))  //unsure of terminating codition
    {
        std::vector<double> sample_node = sample(); // we're sure its in free space
        int nearest_node = nearest(tree, sample_node);
        steer_node = steer(tree[nearest_node], sample_node);
        // check if steer node is free then push back
        if(is_xy_occupied(steer_node.x, steer_node.y))
        {
            continue;
        }
        steer_node.is_root = false;
        steer_node.index = count;
        tree.push_back(steer_node);
        count++;
        // cout << "sample node: " << sample_node[0] << ", " << sample_node[1] << endl;
        // cout << "steer node: " << steer_node.x << ", " << steer_node.y << endl;
        // cout << "tree end: " << tree.back().x << ", " << tree.back().y << endl;
        // cout << "count: " << count << endl;
    }
    cout << "Reached goal" << endl;
    // cout << "tree size: " << tree.size() << endl;
    rrt_path = find_path(tree, steer_node);
    publish_drive();
}

void RRT::waypoint_callback(const interfaces_hot_wheels::msg::Waypoint::ConstSharedPtr waypoint) {
    /*
    The wpt callback, where we determine what waypoint to make the goal in RRT projected onto
    the occupancy grid if it is far away
    Args:
       wpt_msg (*Waypoint): pointer to the incoming waypoint message
    Returns:
        void
        populated member var goal_*: the goal point
    */
    
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.id = marker_id;
    goal_marker.scale.x = 0.15;
    goal_marker.scale.y = 0.15;
    goal_marker.scale.z = 0.15;
    goal_marker.color.a = 0.5;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0;
    goal_marker.header.frame_id = local_frame;
    
    // check if the waypoint is outside the grid
    float pt_dist = std::sqrt(std::pow(waypoint->x, 2) + std::pow(waypoint->y, 2));
    // use similar triangles to get the distance from the origin of the car projected on the grid
    float y_dist_temp = this->get_parameter("height_m").get_parameter_value().get<float>() * waypoint->y / std::abs(waypoint->x);
    // get the hypotenuse in the grid along the unit vector to the waypoint
    float grid_dist = std::sqrt(std::pow(this->get_parameter("height_m").get_parameter_value().get<float>(), 2) +
                                std::pow(y_dist_temp, 2));
    if (pt_dist > grid_dist) {
        // project onto the grid since outside of the grid
        goal_x = this->get_parameter("height_m").get_parameter_value().get<float>();
        goal_y = y_dist_temp;
    } else {
        // already inside of the grid
        goal_x = waypoint->x;
        goal_y = waypoint->y;
    }

    goal_marker.pose.position.x = goal_x;
    goal_marker.pose.position.y = goal_y;
    rrt_goal_vis_pub_->publish(goal_marker);
    
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


    std::vector<double> sampled_point;
    bool terminate_flag = false;
    while (!terminate_flag) {
        float x_samp = x_dist(gen);
        float y_samp = y_dist(gen);

        // check if in the free space
        if (is_xy_occupied(x_samp, y_samp)) {
            continue; // is an obstacle
        } else {
            // is free space
            sampled_point.push_back(x_samp);
            sampled_point.push_back(y_samp);
            terminate_flag = true;
        }
    }

    return sampled_point;
}

void RRT::publish_drive(){    
    /*
    This method publishes the drive message to the drive topic
    Checks waypoint on RRT path just outside lookahead distance
    */
    
    double theta; 
    for (auto waypoint : rrt_path)
    {
        double dist = sqrt(pow(waypoint.x, 2) + pow(waypoint.y,2));
        if (dist > this->get_parameter("L").get_parameter_value().get<float>())
        {
            theta = 2 * waypoint.y/pow(dist, 2);
            break;
        }
    }
    
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.steering_angle = this->get_parameter("Kp").get_parameter_value().get<float>() * theta;
    drive_msg.drive.speed = this->get_parameter("v").get_parameter_value().get<float>();
    drive_pub_->publish(drive_msg);

    cout << "Published drive message" << endl;
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
    for (auto node : tree)
    {
        double x = node.x - sampled_point[0];
        double y = node.y - sampled_point[1];
        double dist = sqrt(pow(x,2) - pow(y,2));
        if(dist < nearest_dist)
        {
            nearest_node = node.index;
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
    double x, y, dist;

    x = sampled_point[0] - nearest_node.x;
    y = sampled_point[1] - nearest_node.y;
    dist = sqrt(pow(x,2) - pow(y,2));

    if (dist > max_expansion_dist)
    {
        double theta = atan2(y,x);
        new_node.x = nearest_node.x + max_expansion_dist * cos(theta);
        new_node.y = nearest_node.y + max_expansion_dist * sin(theta);
    }
    else
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    new_node.parent = nearest_node.index;

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
    // TODO: consider the cars dims with AABB
    // use AABB (axis-aligned bounding box)
    // refer here: https://www.realtimerendering.com/intersections.html

    float dist = sqrt(pow(nearest_node.x - new_node.x, 2) + pow(nearest_node.y - new_node.y, 2));
    float unit_vec_x = (new_node.x - nearest_node.x) / dist;
    float unit_vec_y = (new_node.y - nearest_node.y) / dist;
    int number_of_steps = 10;
    float d_dist = dist / number_of_steps;
    float d_x = unit_vec_x * d_dist;
    float d_y = unit_vec_y * d_dist;
    float x = nearest_node.x;
    float y = nearest_node.y;

    for(int i = 0; i < number_of_steps; i++)
    {
        x += d_x;
        y += d_y;
        if (is_xy_occupied(x, y)) {
            collision = true;
            break;
        }
    }

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node) {
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
    // cout << "dist: " << dist << endl;
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
    RRT_Node curr_node = latest_added_node;
    cout << "Parent: " << curr_node.parent << endl;
    // cout << "Tree size: " << tree.size() << endl;
    // int count = 0;

    visualization_msgs::Marker marker_path;
    marker_path.header.frame_id = local_frame;
    marker_path.action = visualization_msgs::Marker::ADD;
    marker_path.type = visualization_msgs::Marker::LINE_LIST;
    marker_path.id = 6;
    marker_path.scale.x = 0.15;
    marker_path.color.a = 0.5;
    marker_path.color.r = 1.0;
    marker_path.color.g = 0.0;
    marker_path.color.b = 0.0;


    while (curr_node.parent != -1)
    {
        found_path.push_back(curr_node);
        geometry_msgs::Point p;
        p.x = curr_node.x;
        p.y = curr_node.y;
        marker_path.points.push_back(p);
        curr_node = tree[curr_node.parent];
        cout << "Parent: " << curr_node.parent << endl;
    }
    cout << "found path" << endl;

    rrt_branch_vis_pub_->publish(marker_path);

    return found_path;
}

bool RRT::is_xy_occupied(float x, float y){
    /*
    This method checks if the given x,y coordinate is occupied
    */
    int pos =  xy2ind(x, y);
    if(occupancy_grid.data[pos] == 100){
        return true;
    }
    else{
        return false;
    }
}

int RRT::xy2ind(float x, float y){
    /*
    This method converts x,y coordinates to an index in the occupancy grid
    */
    int x_ind = (int) (floor(y) - occupancy_grid.info.width/2);
    int y_ind = (int) (floor(x));
    int pos = (int) (x_ind * occupancy_grid.info.width + y_ind);
    return pos;
}

// RRT* methods
/*
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returngen((std::random_device())()s the cost associated with a node
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
*/