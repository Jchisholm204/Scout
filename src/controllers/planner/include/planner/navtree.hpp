/**
 * @file navtree.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief Navigation Tree
 * @version 0.1
 * @date Created: 2026-03-16
 * @modified Last Modified: 2026-03-16
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _NAV_TREE_HPP_
#define _NAV_TREE_HPP_

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/clock.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

// All points in here are global points
class NavTree {
  public:
    typedef struct nav_node {
        struct nav_node* parent;
        std::vector<struct nav_node*> children;
        double x, y;
        bool visited;
    } nav_node_t;
    NavTree(double conn_radi);
    ~NavTree();
    // Returns ptr if a new node was added, null if it already exists
    nav_node_t* add_node(geometry_msgs::msg::Point new_point, nav_node_t* parent);
    // Returns the previous visited value of the point
    bool set_visited(nav_node_t* pNode, bool visited_state);
    // Returns the current value of the points visited
    bool get_visited(nav_node_t* pNode);
    // True if node exists and was removed
    bool remove_node(nav_node_t* pNode);

    nav_node_t* get_nearest(geometry_msgs::msg::Point);
    nav_node_t* get_paernt(nav_node_t* pNode) { return pNode->parent; }

    // Returns how many nodes are in the tree
    std::size_t get_n(void) { return _all_nodes.size(); }

    // Returns a viz msg of all points offset from the local frame
    const visualization_msgs::msg::MarkerArray get_all(
        geometry_msgs::msg::Point drone_pos, double drone_yaw, std::string frame_id);

  private:
    nav_node_t _base_node;
    double _conn_radi;
    std::vector<nav_node_t*> _all_nodes;
};

#endif
