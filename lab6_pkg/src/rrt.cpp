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
            pose_topic("/pf/viz/inferred_pose"), scan_topic("/scan"), cur_wpt_topic("/waypoint"),
            drive_topic("/drive"), occ_grid_topic("/occ_grid"), local_frame("/ego_racecar/laser_model"),
            grid_res_m(0.1), grid_width_m(2.0), grid_height_m(3.0), inflate(2), MAX_ITER(1000)
{
    // // ROS topics
    // pose_topic = "/pf/viz/inferred_pose";
    // scan_topic = "/scan";
    // cur_wpt_topic = "/waypoint";
    // drive_topic = "/drive";
    // occ_grid_topic = "/occ_grid";
    // local_frame = "/ego_racecar/laser_model";

    // ROS publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
    occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occ_grid_topic, 10);

    // ROS params
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Lookahead distance in meters";
    this->declare_parameter("L", 1.0, param_desc);
    param_desc.description = "Kp value";
    this->declare_parameter("Kp", 0.3);
    param_desc.description = "Velocity";
    this->declare_parameter("v", 0.5);

    // // TODO: get the actual car width
    // param_desc.description = "Width in the car in meters";
    // this->declare_parameter("car_width", .15, param_desc);

    this->goal_x = grid_height_m;
    goal_threshold = 0.1;//grid_res_m / 2.0;
    max_expansion_dist = 0.1;//grid_res_m / 10.0;
    neighbor_threshold = 0.05;

    // ROS subscribers
    // TODO: create subscribers as you need
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    waypoint_sub_ = this->create_subscription<interfaces_hot_wheels::msg::Waypoint>(
      cur_wpt_topic, 1, std::bind(&RRT::waypoint_callback, this, std::placeholders::_1));

    // random generator
    std::uniform_real_distribution<> x_temp(
        0.0,
        grid_height_m);
    std::uniform_real_distribution<> y_temp(
        -grid_width_m/2.0,
         grid_width_m/2.0);
    x_dist.param(x_temp.param());
    y_dist.param(y_temp.param());

    // occupancy grid
    occupancy_grid.header.frame_id = local_frame;
    occupancy_grid.info.origin.position.x = 0.0;
    occupancy_grid.info.origin.position.y = grid_width_m/2.0;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.x = 0.0;
    occupancy_grid.info.origin.orientation.y = 0.0;
    occupancy_grid.info.origin.orientation.z = -sqrt(2.0)/2.0;
    occupancy_grid.info.origin.orientation.w = sqrt(2.0)/2.0;

    occupancy_grid.info.resolution = grid_res_m;
    float width = (grid_width_m / occupancy_grid.info.resolution);
    float height = (grid_height_m / occupancy_grid.info.resolution);
    occupancy_grid.info.width =  width;
    occupancy_grid.info.height = height;
    grid_theta = atan2(height, width/2.0);

    // store the top corner distances (max possible)
    max_occ_dist = sqrt(pow(grid_width_m, 2)/4.0 + pow(grid_height_m, 2));

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");

    // visualization code
    rrt_goal_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_goal", 1);
    rrt_node_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_nodes", 1);
    rrt_path_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_path", 1);
    // rrt_branch_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_branch", 1);
    rrt_branch_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_branch", 1);
    rrt_cur_waypoint_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_waypoint", 1);


    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.id = marker_id;
    goal_marker.scale.x = 0.15;
    goal_marker.scale.y = 0.15;
    goal_marker.scale.z = 0.15;
    goal_marker.color.a = 0.5;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0;
    goal_marker.header.frame_id = local_frame;

    node_marker.header.frame_id = local_frame;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.type = visualization_msgs::msg::Marker::POINTS;
    node_marker.id = 1002;
    node_marker.scale.x = 0.03;
    node_marker.scale.y = 0.03;
    node_marker.scale.z = 0.03;
    node_marker.color.a = 1.0;
    node_marker.color.r = 1.0;   
    node_marker.color.g = 0.0;
    node_marker.color.b = 0.0;

    path_marker.header.frame_id = local_frame;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    path_marker.id = 1000;
    path_marker.scale.x = 0.08;
    path_marker.color.a = 0.8;
    path_marker.color.r = 0.0;   
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;


    // branch_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    // branch_marker.id = 2000;
    // branch_marker.scale.x = 0.1;
    // branch_marker.color.a = 0.5;
    // branch_marker.color.r = 0.0;
    // branch_marker.color.g = 0.0;
    // branch_marker.color.b = 1.0;
    // branch_marker.header.frame_id = local_frame;


    rrt_cur_waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
    rrt_cur_waypoint_marker.id = 1001;
    rrt_cur_waypoint_marker.scale.x = 0.1;
    rrt_cur_waypoint_marker.scale.y = 0.1;
    rrt_cur_waypoint_marker.scale.z = 0.1;
    rrt_cur_waypoint_marker.color.a = 1.0;
    rrt_cur_waypoint_marker.color.r = 0.1;
    rrt_cur_waypoint_marker.color.g = 0.1;
    rrt_cur_waypoint_marker.color.b = 0.1;
    rrt_cur_waypoint_marker.header.frame_id = local_frame;
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
    // occupancy_grid.info.map_load_time = scan_msg->header.stamp;
    // occupancy_grid.header.stamp = scan_msg->header.stamp;
    // occupancy_grid.header.frame_id = local_frame;

    occupancy_grid.data.clear();
    for (int it=0;it<occupancy_grid.info.width*occupancy_grid.info.height;it++)
    {  
        occupancy_grid.data.push_back(0);
    }
    // for(int it=0;it<occupancy_grid.info.width*occupancy_grid.info.height;it+=occupancy_grid.info.width)
    // {  
    //     occupancy_grid.data[it]=25; // y is lighter
    // }
    // for(int it=0;it<occupancy_grid.info.width;it++)
    // {  
    //     occupancy_grid.data[it] = 75; // x is darker
    // }

    float rad90 = 90.0 * M_PI / 180.0;
    int index_neg90 = (int) ((-rad90 - scan_msg->angle_min) / scan_msg->angle_increment);
    int index_pos90 = (int) ((rad90 - scan_msg->angle_min) / scan_msg->angle_increment);
    // cout << "index neg90: " << index_neg90 << endl;
    // cout << "index pos90: " << index_pos90 << endl;

    for (int i=index_neg90; i< index_pos90; i++)
    {  
        // cout << "i: " << i << endl;
        if (scan_msg->ranges[i] < max_occ_dist)
        {
            // cout << i << endl;
            // forward and right to left on the grid
            // float fwd = (scan_msg->ranges[i] * cos(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution);
            // float rtl = (scan_msg->ranges[i] * sin(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution);
            
            float fwd = scan_msg->ranges[i] * cos(scan_msg->angle_min + i * scan_msg->angle_increment);
            float rtl = scan_msg->ranges[i] * sin(scan_msg->angle_min + i * scan_msg->angle_increment);
            
            // the origin of the grid is the bottom left so adjust from car frame calcs
            // cout << "angle" << (scan_msg->angle_min + i * scan_msg->angle_increment)*(180.0/M_PI) << endl;
            // cout << (scan_msg->ranges[i] * sin(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution) << " rtl: " << rtl << endl;
            // cout << (scan_msg->ranges[i] * cos(scan_msg->angle_min + i * scan_msg->angle_increment) / occupancy_grid.info.resolution) << " fwd: " << fwd << endl;
            if (fwd < grid_height_m && abs(rtl) < grid_width_m/2.0) {
                // float y = -(rtl - occupancy_grid.info.width/2);
                // float x = fwd;

                // cout << "x: " << x << endl;
                // cout << "y: " << y << endl;
                occupancy_grid.data[xy_to_1d(fwd, rtl)] = 100;

                // inflate the obstacles
                std::array<int,2> xy2d = xy_to_2d(fwd, rtl);
                inflate_obstacles(xy2d[0], xy2d[1]);
            }
        }
    }
    
    // cout << "occ grid ready" << endl;
    occ_grid_pub_->publish(occupancy_grid);
    // cout << "scan callback done" << endl;
}

void RRT::inflate_obstacles(int x, int y) {
    for (int i = -inflate; i <= inflate; i++) {
        for (int j = -inflate; j <= inflate; j++) {
            // check if it is a valid cell 
            if ((x + i) >= 0 &&
                (x + i) < occupancy_grid.info.height  &&
                (y + j) >= 0 &&
                (y + j) < occupancy_grid.info.width) {
                    occupancy_grid.data[(x + i)*occupancy_grid.info.width + (y + j)] = 100;
            }    
        }
    }
};

void RRT::pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) {
    /*
    The pose callback when subscribed to particle filter's inferred pose
    The RRT main loop happens here
    Args:
       pose_msg (*PoseStamped): pointer to the incoming pose message
    Returns:
       tree as std::vector
    */
    cout << "pose callback lol" << endl;
    // occupancy_grid.info.origin = pose_msg->pose;
    // occupancy_grid.info.origin.position.x = pose_msg->pose.position.x;
    // occupancy_grid.info.origin.position.y = pose_msg->pose.position.y + occupancy_grid.info.width/2 * occupancy_grid.info.resolution;
    // cout << "origin: " << occupancy_grid.info.origin.position.x << ", " << occupancy_grid.info.origin.position.y << endl;
    
    tree.clear();
    
    root.x = 0;
    root.y = 0;
    root.cost = 0.0;
    root.parent_idx = -1;
    root.is_root = true;
    root.index = 0;

    tree.push_back(root);
    RRT_Node steer_node;
    int count = 1;
    int iter = 0;
    while (!is_goal(tree.back()) && iter < MAX_ITER)  //unsure of terminating codition
    {
        iter++;
        std::vector<double> sample_node = sample(); // we're sure its in free space
        RRT_Node nearest_node = nearest(tree, sample_node);
        steer_node = steer(nearest_node, sample_node);
        if(is_xy_occupied(steer_node.x, steer_node.y))
        {
            // check if steer node is free
            continue;
        }
        if(check_collision(nearest_node, steer_node))
        {
            // check for collision between nearest node and steer node (sample node, but closer)
            continue;
        }
        
        
        steer_node.is_root = false;
        steer_node.index = count;
        tree.push_back(steer_node);
        nearest_node.children_idx.push_back(steer_node.index);
        count++;

        // cout << "sample node: " << sample_node[0] << ", " << sample_node[1] << endl;
        // cout << "steer node: " << steer_node.x << ", " << steer_node.y << endl;
        // cout << "tree end: " << tree.back().x << ", " << tree.back().y << endl;
        // cout << "count: " << count << endl;
    }
    // cout << "Reached goal" << endl;
    // cout << "tree size: " << tree.size() << endl;
    rrt_path = find_path(tree, steer_node);
    visualize_tree(tree);
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

    if (waypoint->x < grid_height_m && abs(waypoint->y) < grid_width_m / 2.0) {
        // waypoint is inside the grid
        goal_x = waypoint->x;
        goal_y = waypoint->y;
    }
    else 
    {
        // waypoint is outside the grid, so we project it onto the grid edge
        // comparision of angle from the origin of the laser to the waypoint 
        // and the angle of the grid corner to decide top or side projection

        float waypoint_theta = atan2(waypoint->x, waypoint->y); // angle from the origin of the laser to the waypoint
        if (abs(waypoint_theta) > grid_theta) {
            // waypoint is projected onto top of occupancy grid
            goal_x = grid_height_m;
            goal_y = grid_height_m * waypoint->y / waypoint->x;
        }

        else {
            // waypoint is projected onto the side of the occupancy grid
            goal_x = (grid_width_m/2.0) * waypoint->x / waypoint->y;
            goal_y = sign(waypoint->y) * grid_width_m / 2.0;
        }
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
    while (true) {
        float x_samp = x_dist(gen);
        float y_samp = y_dist(gen);

        // check if in the free space
        if (is_xy_occupied(x_samp, y_samp)) {
            continue; // is an obstacle
        } else {
            // is free space
            sampled_point.push_back(x_samp);
            sampled_point.push_back(y_samp);
            break;
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
            rrt_cur_waypoint_marker.pose.position.x = waypoint.x;
            rrt_cur_waypoint_marker.pose.position.y = waypoint.y;
            break;
        }
    }
    
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.steering_angle = this->get_parameter("Kp").get_parameter_value().get<float>() * theta;
    drive_msg.drive.speed = this->get_parameter("v").get_parameter_value().get<float>();
    drive_pub_->publish(drive_msg);
    rrt_cur_waypoint_vis_pub_->publish(rrt_cur_waypoint_marker);

    // cout << "Published drive message" << endl;

}

RRT_Node RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    /*
    This method returns the nearest node on the tree to the sampled point
    Args:
        tree (std::vector<RRT_Node>): the current RRT tree
        sampled_point (std::vector<double>): the sampled point in free space
    Returns:
        nearest_node (int): index of nearest node on the tree
    */

    RRT_Node nearest_node = tree.front();
    double nearest_dist = MAXFLOAT;
    for (auto node : tree)
    {
        double x = node.x - sampled_point[0];
        double y = node.y - sampled_point[1];
        double dist = sqrt(pow(x,2) - pow(y,2));
        if(dist < nearest_dist)
        {
            nearest_node = node;
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
    new_node.parent_idx = nearest_node.index;

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
    int number_of_steps = 15;
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
    // cout << "parent_idx: " << curr_node.parent_idx << endl;
    // cout << "Tree size: " << tree.size() << endl;
    // int count = 0;

    int ctr = 0;

    path_marker.points.clear();

    geometry_msgs::msg::Point p;
    p.x = curr_node.x;
    p.y = curr_node.y;
    path_marker.points.push_back(p);


    while (curr_node.parent_idx != -1)
    {
        found_path.push_back(curr_node);

        p.x = curr_node.x;
        p.y = curr_node.y;
        path_marker.points.push_back(p);
        path_marker.points.push_back(p);

        curr_node = tree[curr_node.parent_idx];
        // cout << "parent_idx: " << curr_node.parent_idx << endl;
        ctr ++; 
    }

    // cout << "found path" << endl;

    p.x = curr_node.x;
    p.y = curr_node.y;
    path_marker.points.push_back(p);

    rrt_path_vis_pub_->publish(path_marker);

    return found_path;
}

// void RRT::visualize_tree(std::vector<RRT_Node> &tree){   

    

//     node_marker.points.clear();
//     branch_marker.points.clear();

//     for (auto node : tree)
//     {
//         geometry_msgs::msg::Point p;
//         p.x = node.x;
//         p.y = node.y;
//         node_marker.points.push_back(p);
//         for (auto child : node.children_idx)
//         {
//             branch_marker.points.push_back(p);
//             geometry_msgs::msg::Point p_child;
//             p_child.x = tree[child].x;
//             p_child.y = tree[child].y;
//             branch_marker.points.push_back(p_child);
//         }

//     }

//     rrt_node_vis_pub_->publish(node_marker);
//     rrt_branch_vis_pub_->publish(branch_marker);
// }


void RRT::visualize_tree(std::vector<RRT_Node> &tree){   

    int m_id = 2000;
    
    visualization_msgs::msg::Marker branch_marker;
    branch_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    branch_marker.id = 2000;
    branch_marker.scale.x = 0.1;
    branch_marker.color.a = 0.5;
    branch_marker.color.r = 0.0;
    branch_marker.color.g = 0.0;
    branch_marker.color.b = 1.0;
    branch_marker.header.frame_id = local_frame;

    node_marker.points.clear();

    for (auto node : tree)
    {
        geometry_msgs::msg::Point p;
        p.x = node.x;
        p.y = node.y;
        node_marker.points.push_back(p);
        for (auto child : node.children_idx)
        {
            branch_marker.points.clear();
            branch_marker.id = m_id++;
            branch_marker.points.push_back(p);
            geometry_msgs::msg::Point p_child;
            p_child.x = tree[child].x;
            p_child.y = tree[child].y;
            branch_marker.points.push_back(p_child);
            branch_marker_arr.markers.push_back(branch_marker);
        }

    }

    rrt_node_vis_pub_->publish(node_marker);
    rrt_branch_vis_pub_->publish(branch_marker_arr);
}


bool RRT::is_xy_occupied(float x, float y){
    /*
    This method checks if the given x,y coordinate is occupied
    */
    int pos =  xy_to_1d(x, y);
    
    if(occupancy_grid.data[pos] == 100){
        // cout << "x: " << x << "; y: " << y << " is occupied" << endl;
        return true;
    }
    else{
        return false;
    }
}

// int RRT::xy2ind(float x, float y){
//     /*
//     This method converts x,y coordinates to an index in the occupancy grid
//     */
//     cout << "x: " << x << "; y: " << y << endl;
//     cout << "x " << x/occupancy_grid.info.resolution << "; y " <<  y/occupancy_grid.info.resolution << endl;
//     int y_ind = -(int) (floor(y/occupancy_grid.info.resolution) - occupancy_grid.info.width/2);
//     int x_ind = (int) (floor(x/occupancy_grid.info.resolution));
//     int pos = (int) (x_ind * occupancy_grid.info.width + y_ind);
//     cout << "x_ind: " << x_ind << "; y_ind: " << y_ind << endl;
//     cout << "pos: " << pos << endl;
//     return pos;
// }

std::array<int,2> RRT::xy_to_2d(float x, float y){
    /*
    This method converts x,y coordinates to an 2D coordinate in the occupancy grid
    */
    cout << "x: " << x << "; y: " << y << endl;
    cout << "x " << x/occupancy_grid.info.resolution << "; y " <<  y/occupancy_grid.info.resolution << endl;

    int x_ind, y_ind;
    y_ind = -(int) (floor(y/occupancy_grid.info.resolution) - occupancy_grid.info.width/2);
    y_ind--;

    x_ind = (int) (floor(x/occupancy_grid.info.resolution));
    // int pos = (int) (x_ind * occupancy_grid.info.width + y_ind);
    cout << "x_ind: " << x_ind << "; y_ind: " << y_ind << endl;
    
    std::array<int,2> xy_ind = {x_ind, y_ind};
    return xy_ind;
}

int RRT::xy_to_1d(float x, float y){
    /*
    This method converts x,y coordinates to an index in the occupancy grid
    */

    std::array<int,2> xy_ind = xy_to_2d(x, y);
    int pos = (int) (xy_ind[0] * occupancy_grid.info.width + xy_ind[1]);
    return pos;
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
    cost = tree[node.parent_idx].cost + line_cost(tree[node.parent_idx], node);

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
    double x, y;

    x = n1.x - n2.x;
    y = n1.y - n2.y;
    cost = sqrt(pow(x,2) + pow(y,2));
    

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
    
    for (auto curr_node : tree)
    {
        if (line_cost(curr_node, node) < neighbor_threshold)
        {
            neighborhood.push_back(curr_node.index);
        }
    } 

    return neighborhood;
}


float sign(float x) {
    return (x > 0) ? 1.0 : ((x < 0) ? -1.0 : 0.0);
}