/**
 * @file navtree.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-03-16
 * @modified Last Modified: 2026-03-16
 *
 * @copyright Copyright (c) 2026
 */

#include "planner/navtree.hpp"

NavTree::NavTree(double conn_radi) : _conn_radi(conn_radi) {
}

NavTree::~NavTree() {
    // Clean up heap allocated nodes to prevent memory leaks
    for (auto node : _all_nodes) {
        delete node;
    }
    _all_nodes.clear();
}

NavTree::nav_node_t* NavTree::get_nearest(geometry_msgs::msg::Point pt) {
    nav_node_t* nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();

    for (auto node : _all_nodes) {
        double dist = std::hypot(node->x - pt.x, node->y - pt.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    return nearest;
}

NavTree::nav_node_t* NavTree::add_node(geometry_msgs::msg::Point new_point,
                                       nav_node_t* parent) {
    // 1. Check if a node already exists within the connection radius
    nav_node_t* nearest = get_nearest(new_point);
    if (nearest) {
        double dist = std::hypot(nearest->x - new_point.x, nearest->y - new_point.y);
        if (dist < _conn_radi) {
            return nullptr; // Merged: Spatial duplicate found
        }
    }

    // 2. Create new node on the heap
    nav_node_t* node = new nav_node_t();
    node->x = new_point.x;
    node->y = new_point.y;
    node->visited = false;
    node->parent = parent;

    if (parent) {
        parent->children.push_back(node);
    }

    _all_nodes.push_back(node);
    return node;
}

bool NavTree::set_visited(nav_node_t* pNode, bool visited_state) {
    if (!pNode)
        return false;
    bool prev = pNode->visited;
    pNode->visited = visited_state;
    return prev;
}

bool NavTree::get_visited(nav_node_t* pNode) {
    return pNode ? pNode->visited : false;
}

bool NavTree::remove_node(nav_node_t* pNode) {
    if (!pNode)
        return false;

    auto it = std::find(_all_nodes.begin(), _all_nodes.end(), pNode);
    if (it != _all_nodes.end()) {
        // Remove from parent's child list
        if (pNode->parent) {
            auto& c = pNode->parent->children;
            c.erase(std::remove(c.begin(), c.end(), pNode), c.end());
        }
        // Note: In a real tree, you'd decide what to do with orphans (children).
        // Here we just delete the node.
        delete *it;
        _all_nodes.erase(it);
        return true;
    }
    return false;
}

const visualization_msgs::msg::MarkerArray NavTree::get_all(
    geometry_msgs::msg::Point drone_pos, double drone_yaw, std::string frame_id) {
    visualization_msgs::msg::MarkerArray markers;
    auto now = rclcpp::Clock().now();

    // Setup Markers (Sphere List for nodes, Line List for edges)
    visualization_msgs::msg::Marker dots, lines;
    dots.header.frame_id = lines.header.frame_id = frame_id;
    dots.header.stamp = lines.header.stamp = now;

    dots.ns = "nav_nodes";
    dots.id = 0;
    dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    dots.scale.x = dots.scale.y = dots.scale.z = 0.2;
    dots.color.a = 1.0;
    dots.color.g = 1.0;

    lines.ns = "nav_edges";
    lines.id = 1;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.scale.x = 0.05;
    lines.color.a = 0.8;
    lines.color.b = 1.0;

    // Pre-calculate trig for performance
    double cos_y = std::cos(drone_yaw);
    double sin_y = std::sin(drone_yaw);

    for (auto node : _all_nodes) {
        // 1. Translate
        double dx = node->x - drone_pos.x;
        double dy = node->y - drone_pos.y;

        // 2. Rotate (Inverse rotation to bring global into local)
        geometry_msgs::msg::Point p;
        p.x = dx * cos_y + dy * sin_y;
        p.y = -dx * sin_y + dy * cos_y;
        p.z = 0.0;

        dots.points.push_back(p);

        if (node->parent) {
            lines.points.push_back(p);

            // Transform parent point same way
            double pdx = node->parent->x - drone_pos.x;
            double pdy = node->parent->y - drone_pos.y;

            geometry_msgs::msg::Point pp;
            pp.x = pdx * cos_y + pdy * sin_y;
            pp.y = -pdx * sin_y + pdy * cos_y;
            pp.z = 0.0;
            lines.points.push_back(pp);
        }
    }

    markers.markers.push_back(dots);
    markers.markers.push_back(lines);
    return markers;
}
