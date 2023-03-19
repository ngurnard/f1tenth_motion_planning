// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "interfaces_hot_wheels/msg/waypoint.hpp"

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    float x, y;
    float cost; // only used for RRT*
    int parent_idx; // index of parent node in the tree vector
    int index; // index of this node in the tree vector
    std::vector<int> children_idx = {-1}; // initialize the child to be -1 (no child)
    bool is_root = false;
} RRT_Node;

static unsigned int marker_id = 0;

class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    // TODO: add the publishers and subscribers you need
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<interfaces_hot_wheels::msg::Waypoint>::SharedPtr waypoint_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid >::SharedPtr occ_grid_pub_;

    // visualization publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_goal_vis_pub_;          // goal 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_node_vis_pub_;     // every node
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_path_vis_pub_;          // final path
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_branch_vis_pub_;        // all branches
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_branch_vis_pub_;        // all branches
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_cur_waypoint_vis_pub_;  // tracked waypoint

    // visualization markers
    visualization_msgs::msg::Marker goal_marker;
    visualization_msgs::msg::Marker node_marker;
    visualization_msgs::msg::Marker path_marker;
    visualization_msgs::msg::MarkerArray branch_marker_arr;
    // visualization_msgs::msg::Marker branch_marker;
    visualization_msgs::msg::Marker rrt_cur_waypoint_marker;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    
    // occupancy grid
    float grid_width_m, grid_height_m, grid_res_m;
    float grid_theta;
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    float max_occ_dist;
    bool is_xy_occupied(float x, float y);
    int xy_to_1d(float x, float y);
    std::array<int, 2> xy_to_2d(float x, float y);
    int inflate;
    void inflate_obstacles(int x, int y);

    // threshold variables
    float goal_threshold;
    float max_expansion_dist;
    float neighbor_threshold;
    int MAX_ITER;

    // tree variables
    std::vector<RRT_Node> tree;
    RRT_Node root;

    // path
    std::vector<RRT_Node> rrt_path;

    // waypoint vars
    float goal_x;
    float goal_y;

    // topic vars
    std::string pose_topic;
    std::string scan_topic;
    std::string cur_wpt_topic;
    std::string drive_topic;
    std::string occ_grid_topic;
    
    // frame vars
    std::string local_frame;

    // callbacks
    // where rrt actually happens
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void waypoint_callback(const interfaces_hot_wheels::msg::Waypoint::ConstSharedPtr waypoint);
    // publish drive
    void publish_drive();
    void visualize_tree(std::vector<RRT_Node> &tree);

    // RRT methods
    std::vector<float> sample();
    RRT_Node nearest(std::vector<RRT_Node> &tree, std::vector<float> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<float> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    // RRT* methods
    float Cost(std::vector<RRT_Node> &tree, RRT_Node &node, int neighbor_idx);
    float line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);
    void dfs(std::vector<RRT_Node> &tree, RRT_Node &node);

};

float sign(float x);

