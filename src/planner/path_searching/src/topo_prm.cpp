#include "path_searching/topo_prm.h"
#include <cmath>
#include <algorithm>

using namespace std;
using namespace Eigen;

namespace ego_planner {

TopoPRM::TopoPRM() 
    : step_size_(0.2), search_radius_(5.0), max_sample_num_(1000), 
      collision_check_resolution_(0.05) {
}

TopoPRM::~TopoPRM() {
}

void TopoPRM::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    topo_paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("topo_paths", 10);
    
    ROS_INFO("[TopoPRM] Initialized with step_size: %f, search_radius: %f", 
             step_size_, search_radius_);
}

bool TopoPRM::searchTopoPaths(const Vector3d& start, const Vector3d& goal,
                             vector<TopoPath>& topo_paths) {
    topo_paths.clear();
    
    // Generate topological paths
    vector<TopoPath> candidate_paths = findTopoPaths(start, goal);
    
    if (candidate_paths.empty()) {
        ROS_WARN("[TopoPRM] No valid topological paths found");
        return false;
    }
    
    // Sort paths by cost
    sort(candidate_paths.begin(), candidate_paths.end(),
         [](const TopoPath& a, const TopoPath& b) {
             return a.cost < b.cost;
         });
    
    topo_paths = candidate_paths;
    
    // Visualize paths
    visualizeTopoPaths(topo_paths);
    
    ROS_INFO("[TopoPRM] Found %zu topological paths", topo_paths.size());
    return true;
}

vector<TopoPath> TopoPRM::findTopoPaths(const Vector3d& start, const Vector3d& goal) {
    vector<TopoPath> paths;
    
    // Direct path (if collision-free)
    vector<Vector3d> direct_path = {start, goal};
    if (isPathValid(direct_path)) {
        double cost = calculatePathCost(direct_path);
        paths.emplace_back(direct_path, cost, 0);
    }
    
    // Find obstacles along direct line
    Vector3d dir = (goal - start).normalized();
    double dist = (goal - start).norm();
    
    vector<Vector3d> obstacle_centers;
    
    // Sample along direct path to find obstacles
    for (double t = step_size_; t < dist; t += step_size_) {
        Vector3d sample_point = start + t * dir;
        if (grid_map_->getInflateOccupancy(sample_point)) {
            obstacle_centers.push_back(sample_point);
        }
    }
    
    // Remove duplicate nearby obstacle centers
    vector<Vector3d> filtered_obstacles;
    for (const auto& obs : obstacle_centers) {
        bool is_duplicate = false;
        for (const auto& filtered : filtered_obstacles) {
            if ((obs - filtered).norm() < search_radius_ * 0.5) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            filtered_obstacles.push_back(obs);
        }
    }
    
    // Generate alternative paths for each obstacle
    int path_id = 1;
    for (const auto& obstacle_center : filtered_obstacles) {
        for (int direction = 0; direction < 4; ++direction) {
            vector<Vector3d> alt_path = generateAlternativePath(start, goal, 
                                                               obstacle_center, direction);
            if (!alt_path.empty() && isPathValid(alt_path)) {
                double cost = calculatePathCost(alt_path);
                paths.emplace_back(alt_path, cost, path_id++);
            }
        }
    }
    
    return paths;
}

vector<Vector3d> TopoPRM::generateAlternativePath(const Vector3d& start,
                                                 const Vector3d& goal,
                                                 const Vector3d& obstacle_center,
                                                 int direction) {
    vector<Vector3d> path;
    
    // Calculate avoidance direction
    Vector3d avoidance_dir;
    Vector3d to_obstacle = obstacle_center - start;
    Vector3d forward_dir = (goal - start).normalized();
    
    switch (direction) {
        case 0: // up
            avoidance_dir = Vector3d(0, 0, 1);
            break;
        case 1: // down  
            avoidance_dir = Vector3d(0, 0, -1);
            break;
        case 2: // left (perpendicular to forward direction)
            avoidance_dir = forward_dir.cross(Vector3d(0, 0, 1)).normalized();
            break;
        case 3: // right
            avoidance_dir = -forward_dir.cross(Vector3d(0, 0, 1)).normalized();
            break;
        default:
            return path; // empty path
    }
    
    // Calculate waypoint to avoid obstacle
    Vector3d waypoint = obstacle_center + avoidance_dir * search_radius_;
    
    // Check if waypoint is valid
    if (grid_map_->getInflateOccupancy(waypoint)) {
        // Try different distances
        for (double dist = search_radius_ * 0.5; dist <= search_radius_ * 2.0; dist += search_radius_ * 0.5) {
            waypoint = obstacle_center + avoidance_dir * dist;
            if (!grid_map_->getInflateOccupancy(waypoint)) {
                break;
            }
        }
    }
    
    // Create path: start -> waypoint -> goal
    path.push_back(start);
    path.push_back(waypoint);
    path.push_back(goal);
    
    return path;
}

bool TopoPRM::isPathValid(const vector<Vector3d>& path) {
    if (path.size() < 2) return false;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (!isLineCollisionFree(path[i], path[i + 1])) {
            return false;
        }
    }
    return true;
}

bool TopoPRM::isLineCollisionFree(const Vector3d& start, const Vector3d& end) {
    Vector3d dir = end - start;
    double dist = dir.norm();
    if (dist < 1e-6) return true;
    
    dir.normalize();
    
    for (double t = 0; t <= dist; t += collision_check_resolution_) {
        Vector3d point = start + t * dir;
        if (grid_map_->getInflateOccupancy(point)) {
            return false;
        }
    }
    return true;
}

double TopoPRM::calculatePathCost(const vector<Vector3d>& path) {
    if (path.size() < 2) return std::numeric_limits<double>::max();
    
    double length_cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length_cost += (path[i + 1] - path[i]).norm();
    }
    
    double smoothness_cost = calculateSmoothnessCost(path);
    double obstacle_cost = calculateObstacleCost(path);
    
    return length_cost + 2.0 * smoothness_cost + 5.0 * obstacle_cost;
}

double TopoPRM::calculateSmoothnessCost(const vector<Vector3d>& path) {
    if (path.size() < 3) return 0.0;
    
    double smoothness_cost = 0.0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Vector3d v1 = (path[i] - path[i - 1]).normalized();
        Vector3d v2 = (path[i + 1] - path[i]).normalized();
        double angle = acos(std::max(-1.0, std::min(1.0, v1.dot(v2))));
        smoothness_cost += angle;
    }
    return smoothness_cost;
}

double TopoPRM::calculateObstacleCost(const vector<Vector3d>& path) {
    double obstacle_cost = 0.0;
    
    for (const auto& point : path) {
        // Check distance to nearest obstacle
        double min_dist = std::numeric_limits<double>::max();
        
        // Sample around the point to find nearest obstacle
        for (double dx = -search_radius_; dx <= search_radius_; dx += step_size_) {
            for (double dy = -search_radius_; dy <= search_radius_; dy += step_size_) {
                for (double dz = -search_radius_; dz <= search_radius_; dz += step_size_) {
                    Vector3d sample = point + Vector3d(dx, dy, dz);
                    if (grid_map_->getInflateOccupancy(sample)) {
                        double dist = Vector3d(dx, dy, dz).norm();
                        min_dist = std::min(min_dist, dist);
                    }
                }
            }
        }
        
        if (min_dist < search_radius_) {
            obstacle_cost += 1.0 / (min_dist + 0.1);
        }
    }
    
    return obstacle_cost;
}

TopoPath TopoPRM::selectBestPath(const vector<TopoPath>& paths) {
    if (paths.empty()) {
        return TopoPath();
    }
    
    // Return the path with minimum cost
    auto best_it = std::min_element(paths.begin(), paths.end(),
        [](const TopoPath& a, const TopoPath& b) {
            return a.cost < b.cost;
        });
    
    return *best_it;
}

void TopoPRM::visualizeTopoPaths(const vector<TopoPath>& paths) {
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = "world";
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Visualize each path with different colors
    for (size_t i = 0; i < paths.size() && i < 10; ++i) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "world";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "topo_paths";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        
        // Different colors for different paths
        if (i == 0) {
            line_marker.color.r = 1.0; line_marker.color.g = 0.0; line_marker.color.b = 0.0;
        } else if (i == 1) {
            line_marker.color.r = 0.0; line_marker.color.g = 1.0; line_marker.color.b = 0.0;
        } else if (i == 2) {
            line_marker.color.r = 0.0; line_marker.color.g = 0.0; line_marker.color.b = 1.0;
        } else {
            line_marker.color.r = 1.0; line_marker.color.g = 0.5; line_marker.color.b = 0.0;
        }
        line_marker.color.a = 0.8;
        line_marker.scale.x = 0.05;
        
        for (const auto& point : paths[i].path) {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
    }
    
    topo_paths_pub_.publish(marker_array);
}

} // namespace ego_planner