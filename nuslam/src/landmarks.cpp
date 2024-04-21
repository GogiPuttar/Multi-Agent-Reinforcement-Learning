/// \file
/// \brief Preforms clustering and circle fitting on the lidar data and then data association for
///        landmark association for EKF SLAM.
///
/// PARAMETERS:
///     \param obstacles.r (double): Radius of cylindrical obstacles [m]
///     \param obstacles.h (double): Height of cylindrical obstacles [m]
///
/// PUBLISHES:
///     \param ~/clusters (visualization_msgs::msg::MarkerArray): Clusters MarkerArray as seen by
///                                                               Clustering algorithm
///     \param ~/circle_fit (visualization_msgs::msg::MarkerArray): Fitted circles MarkerArray as
///                                                                 seen by circle fitting algorithm
///
/// SUBSCRIBES:
///     \param /nusim/fake_lidar_scan (sensor_msgs::msg::LaserScan): Subscribes to the fake lidar
///                                                                  points
///
/// SERVERS:
///     None
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include "turtlelib/circle_fitting.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/// \brief Preforms clustering and circle fitting on the lidar data and then data association for
///        landmark association for EKF SLAM.
///
///  \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
///  \param obstacles_h_ (double): Height of cylindrical obstacles [m]

class landmarks : public rclcpp::Node
{
public:
  landmarks()
  : Node("landmarks")
  {
    // Parameter descirption
    auto obstacles_r_des = rcl_interfaces::msg::ParameterDescriptor{};
    obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
    // Declare default parameters values
    declare_parameter("obstacles.r", 0.0, obstacles_r_des);
    declare_parameter("real_lidar", false);

    // Get params - Read params from yaml file that is passed in the launch file
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    const auto real_lidar = get_parameter("real_lidar").as_bool();
    // RCLCPP_INFO_STREAM(get_logger(), "Real Lidar: " << real_lidar);

    // Subscribers
    if (real_lidar) 
    {
      // need appropriate qos parameter
      lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(
          &landmarks::lidar_callback, this,
          std::placeholders::_1));
    } 
    else 
    {
      // this is just the lidar from nusim which has standard qos
      lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/nusim/fake_lidar_scan", 10, std::bind(
        &landmarks::lidar_callback,
        this, std::placeholders::_1));
    }

    check_yaml_params();

    // Publishers
    cluster_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/clusters", 10);
    circle_fit_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/circle_fit", 10);
  }

private:
  // Variables
  bool clustering_flag = true; // True while building a cluster
  double threshold_dist_ = 0.1; // Threshold for clustering lidar data
  double threshold_points_ = 2; // Threshold for clustering lidar data
  double obstacles_r_ = -1.0;    // Size of obstacles
  double obstacles_h_ = 0.25;
  double lidar_height_ = 0.182;

  // Create objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr circle_fit_publisher_;

  /// \brief Lidar sensor topic callback
  void lidar_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<std::vector<turtlelib::Point2D>> clusters; // Vector of clusters
    bool Flag_cluster_wrap_around = false; // Flag that notifies if wrap around of a cluster has occured

    for (size_t i = 0; i < msg.ranges.size(); i++) { //  Loop through one cycle of lidar points
      if (msg.ranges.at(i) > 0.01) { // Check if the lidar point hit an object
        std::vector<turtlelib::Point2D> temp_cluster{};     // Create a possible cluster
        clustering_flag = true;
        size_t count = 0;

        while (clustering_flag) {
          turtlelib::Point2D point_i = PolarToCartesian(
            msg.ranges.at(
              (i + count) % 360),
            turtlelib::normalize_angle(((i + count) % 360) * turtlelib::PI / 180.0));
          if (count == 0) {
            temp_cluster.push_back(point_i);
          }
          turtlelib::Point2D point_next_i = PolarToCartesian(
            msg.ranges.at(
              (i + 1 + count) % 360),
            turtlelib::normalize_angle(((i + 1 + count) % 360) * turtlelib::PI / 180.0));
          double distance_to_next = turtlelib::magnitude(point_i - point_next_i);

          if (distance_to_next < threshold_dist_) {
            // Save to vector
            temp_cluster.push_back(point_next_i);
            // Check next point in lidar scan
            count++;
          } else if (count > threshold_points_) { // We have atleast 4 points in the cluster
            // End of cluster
            clustering_flag = false;
            i = i + count - 1;

            // Save cluster to list of clusters
            clusters.push_back(temp_cluster);
          } else {
            clustering_flag = false;
          }

          // Check if wrap around of clusters has occured > 360 Degree
          if (i + 1 + count >= 360) {
            Flag_cluster_wrap_around = true;
          }
        }
      }
    }

    // If wrap around has occured check if the last cluster contains the first cluster
    if (Flag_cluster_wrap_around == true) {
      if (clusters.at(0).back().x == clusters.back().back().x &&
        clusters.at(0).back().y == clusters.back().back().y)                                                            //  Only delete if if there is an encapsuled cluster
      {
        clusters.at(0) = clusters.back();     // Save last cluster as first
        clusters.pop_back();     // Delete last cluster
      }

      // TODO - Better than checking exact values above???????
      // if (euclidean_distance(clusters.at(0).back().x, clusters.at(0).back().y, clusters.back().back().x, clusters.back().back().y) < threshold_dist_)
      // {
      //     clusters.at(0) = clusters.back(); // Save last cluster as first
      //     clusters.pop_back(); // Delete last cluster
      // }
    }

    circle_fit(clusters); // Fit circle to clusters
    create_clusters_array(clusters); // Create and publish clusters
  }

  // /// \brief Lidar sensor topic callback
  // void lidar_callback(const sensor_msgs::msg::LaserScan & msg)
  // {
  //   std::vector<std::vector<turtlelib::Point2D>> clusters; // Vector of clusters

  //   //  Loop through one cycle of lidar points
  //   for (size_t i = 0; i < msg.ranges.size(); i++) 
  //   { 
  //     // Check if the lidar point hit an object
  //     if (msg.ranges.at(i) > 0.01) 
  //     { 
  //       // Initialize a possible cluster
  //       std::vector<turtlelib::Point2D> temp_cluster{};     
  //       clustering_flag = true;
  //       size_t count = 0;

  //       while (clustering_flag) 
  //       {
  //         turtlelib::Point2D point_i = PolarToCartesian(
  //                                                       msg.ranges.at((i + count) % 360),
  //                                                       turtlelib::normalize_angle(turtlelib::deg2rad((i + count) % 360))
  //                                                       );

  //         // Initialize first point in possible cluster
  //         if (count == 0) 
  //         {
  //           temp_cluster.push_back(point_i);
  //         }

  //         // Calculate distance from next point
  //         turtlelib::Point2D point_next_i = PolarToCartesian(
  //           msg.ranges.at(
  //             (i + 1 + count) % 360),
  //           turtlelib::normalize_angle(((i + 1 + count) % 360) * turtlelib::PI / 180.0));
  //         double distance_to_next = turtlelib::magnitude(point_next_i - point_i);

  //         // If the next point is sufficiently close, add it to the cluster, and move on to the next point
  //         if (distance_to_next < threshold_dist_) 
  //         {
  //           temp_cluster.push_back(point_next_i);
  //           count++;
  //         } 
  //         // Otherwise, classify what we have up till now as a cluster, if there are enough points in it (removes outliers)
  //         // And then jump the index to the next cluster's first point
  //         else if (count >= threshold_points_) 
  //         { 
  //           clustering_flag = false;
  //           i = i + count - 1;

  //           // Save cluster to list of clusters
  //           clusters.push_back(temp_cluster);
  //         } 
  //         else 
  //         {
  //           clustering_flag = false;
  //         }

  //         // Check if wrap around of clusters will occur ( > 360 degrees)
  //         if (i + count >= 360) 
  //         {
  //           clustering_flag = false;
  //           // Save last cluster to list of clusters
  //           clusters.push_back(temp_cluster);

  //           // If last and first cluster are connected, save last cluster as first and delete last cluster.
  //           if (turtlelib::magnitude(clusters.back().back() - clusters.at(0).at(0)))
  //           {
  //             clusters.at(0) = clusters.back();     
  //             clusters.pop_back();     
  //           }
  //         }
  //       }
  //     }
  //   }
  //   circle_fit(clusters); // Fit circle to clusters
  //   create_clusters_array(clusters); // Create and publish clusters
  // }

  // CITATION BEGINS ------------------------ https://github.com/Marnonel6/EKF_SLAM_from_scratch/blob/main/nuslam/src/landmarks.cpp 
  /// \brief Run the circle fitting algorithm to get the circle radius and location (x,y)
  void circle_fit(std::vector<std::vector<turtlelib::Point2D>> clusters)
  {
    std::vector<turtlelib::Circle> circle_list{};

    double radius_tolerance = 0.01;

    // Iterate through clusters and pass to circle fitting function
    for (size_t i = 0; i < clusters.size(); i++) {
      turtlelib::Circle circle_params = turtlelib::circle_fitting(clusters.at(i));
      if (circle_params.R < 0.1 && circle_params.R > 0.01) { // Filter circle for radii smaller than 0.1 and greater than 0.01
        if(fabs(circle_params.R - obstacles_r_) <= radius_tolerance)
        {
          circle_list.push_back(circle_params);
        }
      }
    }

    create_circles_array(circle_list); // Publish fitted circles as a MarkerArray

  }
  // CITATION ENDS

  /// \brief Create circle fitted MarkerArray as seen by Circle fitting algorithm and publish them to a topic to display them in Rviz
  void create_circles_array(std::vector<turtlelib::Circle> circle_list)
  {
    visualization_msgs::msg::MarkerArray circles_;

    // turtlelib::Point2D landmark_position{circle_list.at(i).x - 0.032*cos(green_turtle_.theta), sensed_landmarks.markers[index].pose.position.y - 0.032*sin(green_turtle_.theta)};

    for (size_t i = 0; i < circle_list.size(); i++) {
      visualization_msgs::msg::Marker circle_;
      circle_.header.frame_id = "green/base_scan";
      circle_.header.stamp = get_clock()->now();
      circle_.id = i;
      circle_.type = visualization_msgs::msg::Marker::CYLINDER;
      circle_.action = visualization_msgs::msg::Marker::ADD;
      circle_.pose.position.x = circle_list.at(i).x;
      circle_.pose.position.y = circle_list.at(i).y;
      circle_.pose.position.z = obstacles_h_ / 2.0 - lidar_height_;
      circle_.pose.orientation.x = 0.0;
      circle_.pose.orientation.y = 0.0;
      circle_.pose.orientation.z = 0.0;
      circle_.pose.orientation.w = 1.0;
      circle_.scale.x = circle_list.at(i).R * 2.0;   //obstacles_r_ * 2.0;       // Diameter in x
      circle_.scale.y = circle_list.at(i).R * 2.0;   //obstacles_r_ * 2.0;       // Diameter in y
      // circle_.scale.x = obstacles_r_ * 2.0;       // Diameter in x
      // circle_.scale.y = obstacles_r_ * 2.0;       // Diameter in y
      circle_.scale.z = obstacles_h_;               // Height
      circle_.color.r = 1.0f; // Yellow
      circle_.color.g = 1.0f;
      circle_.color.b = 0.0f;
      circle_.color.a = 1.0;
      circles_.markers.push_back(circle_);
    }
    circle_fit_publisher_->publish(circles_);
  }

  /// \brief Create clusters MarkerArray as seen by Clustering algorithm and publish them to a topic to display them in Rviz
  void create_clusters_array(std::vector<std::vector<turtlelib::Point2D>> clusters)
  {
    // // Offset between LIDAR and Footprint (fixed, unless things go very ugly)
    // turtlelib::Pose2D lidar_pose_{turtle_.pose().theta, turtle_.pose().x - 0.032*cos(turtle_.pose().theta), turtle_.pose().y - 0.032*sin(turtle_.pose().theta)};
    
    visualization_msgs::msg::MarkerArray clusters_;

    for (size_t i = 0; i < clusters.size(); i++) 
    {
      double x_avg = 0.0;
      double y_avg = 0.0;
      double num_elements = 0.0;
      for (size_t j = 0; j < clusters.at(i).size(); j++) 
      {
        x_avg += clusters.at(i).at(j).x;
        y_avg += clusters.at(i).at(j).y;
        num_elements += 1.0;
      }
      x_avg /= num_elements;
      y_avg /= num_elements;

      visualization_msgs::msg::Marker cluster_;
      cluster_.header.frame_id = "green/base_scan";
      cluster_.header.stamp = get_clock()->now();
      cluster_.id = i;
      cluster_.type = visualization_msgs::msg::Marker::SPHERE;
      cluster_.action = visualization_msgs::msg::Marker::ADD;
      cluster_.pose.position.x = x_avg;
      cluster_.pose.position.y = y_avg;
      cluster_.pose.position.z = 0.0;
      cluster_.pose.orientation.x = 0.0;
      cluster_.pose.orientation.y = 0.0;
      cluster_.pose.orientation.z = 0.0;
      cluster_.pose.orientation.w = 1.0;
      cluster_.scale.x = obstacles_r_ * 2.0;         // Diameter in x
      cluster_.scale.y = obstacles_r_ * 2.0;         // Diameter in y
      cluster_.scale.z = obstacles_r_ * 2.0;               // Height
      cluster_.color.r = 1.0f;                       // Magenta
      cluster_.color.g = 0.0f;
      cluster_.color.b = 1.0f;
      cluster_.color.a = 1.0;
      clusters_.markers.push_back(cluster_);

    }
    cluster_publisher_->publish(clusters_);
  }

  /// \brief Ensures all values are passed via .yaml file
  void check_yaml_params()
  {
    if (  obstacles_r_ == -1.0 
          )
    {
      RCLCPP_ERROR(this->get_logger(), "Param obstacles_r_: %f", obstacles_r_);
      
      throw std::runtime_error("Missing necessary parameters in diff_params.yaml!");
    }

    if (  obstacles_r_ <= 0.0 
          )
    {
      RCLCPP_ERROR(this->get_logger(), "Param obstacles_r_: %f", obstacles_r_);
      
      throw std::runtime_error("Incorrect params in diff_params.yaml!");
    }
  }

  /// \brief Calculate the x,y coordinates from range and bearing
  /// \param range distance to point [m] (double)
  /// \param theta angle to point [radians] (double)
  /// \return 2D vector x and y (turtlelib::Vector2D)
  turtlelib::Point2D PolarToCartesian(double range, double theta)
  {
    return {range * cos(theta), range * sin(theta)};
  }

};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<landmarks>());
  rclcpp::shutdown();
  return 0;
}