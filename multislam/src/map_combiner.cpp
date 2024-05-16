#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class Map_Combiner : public rclcpp::Node
{
public:
  Map_Combiner()
    : Node("map_combiner")
    {
        // Parameter description
        auto num_robots_des = rcl_interfaces::msg::ParameterDescriptor{};

        num_robots_des.description = "number of agents";

        declare_parameter("num_robots", 0, num_robots_des);     // 1,2,3,..

        num_robots_ = get_parameter("num_robots").get_parameter_value().get<int>();

        // Create /proposed_simplified_map publisher
        proposed_simplified_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/proposed_simplified_map", 10);
        
        // Create /true_simplified_map subscriber
        true_simplified_map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/true_simplified_map", 10, std::bind(
            &Map_Combiner::true_simplified_map_callback, this,
            std::placeholders::_1));

        // Create color/map subscriber
        // Literally the least problematic way to do this
        cyan_map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "cyan/map", 10, std::bind(
            &Map_Combiner::cyan_map_callback, this,
            std::placeholders::_1));
        magenta_map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "magenta/map", 10, std::bind(
            &Map_Combiner::magenta_map_callback, this,
            std::placeholders::_1));
        yellow_map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "yellow/map", 10, std::bind(
            &Map_Combiner::yellow_map_callback, this,
            std::placeholders::_1));
        red_map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "red/map", 10, std::bind(
            &Map_Combiner::red_map_callback, this,
            std::placeholders::_1));
        green_map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "green/map", 10, std::bind(
            &Map_Combiner::green_map_callback, this,
            std::placeholders::_1));
        blue_map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "blue/map", 10, std::bind(
            &Map_Combiner::blue_map_callback, this,
            std::placeholders::_1));
    } 

private:

    // Flag to check metadata initialization from true_simplified_map
    int num_robots_;
    std::vector<std::string> colors_ = {"cyan", "magenta", "yellow", "red", "green", "blue"};
    bool initialization_flag = false;

    // Initialize simplified maps
    nav_msgs::msg::OccupancyGrid true_simplified_map_;
    nav_msgs::msg::OccupancyGrid proposed_simplified_map_;

    // Initialize local maps
    std::vector<nav_msgs::msg::OccupancyGrid> local_maps_;

    // Create Objects
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr proposed_simplified_map_publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr true_simplified_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cyan_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr magenta_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr yellow_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr red_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr green_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr blue_map_subscriber_;

    void true_simplified_map_callback(const nav_msgs::msg::OccupancyGrid & msg)
    {   
        if (!initialization_flag) // Initialize only if not yet initialized
        {
            proposed_simplified_map_.header.stamp = get_clock()->now();
            proposed_simplified_map_.header.frame_id = msg.header.frame_id;
            proposed_simplified_map_.info.map_load_time = get_clock()->now();
            proposed_simplified_map_.info.resolution = msg.info.resolution; // meters/cell
            proposed_simplified_map_.info.width = msg.info.width;  // cells
            proposed_simplified_map_.info.height = msg.info.height;  // cells
            proposed_simplified_map_.info.origin.position.x = msg.info.origin.position.x; // meters
            proposed_simplified_map_.info.origin.position.y = msg.info.origin.position.y; // meters
            proposed_simplified_map_.info.origin.position.z = msg.info.origin.position.z; // meters
            proposed_simplified_map_.info.origin.orientation.x = msg.info.origin.orientation.x;
            proposed_simplified_map_.info.origin.orientation.y = msg.info.origin.orientation.y;
            proposed_simplified_map_.info.origin.orientation.z = msg.info.origin.orientation.z;
            proposed_simplified_map_.info.origin.orientation.w = msg.info.origin.orientation.w;

            // Initialize as empty map (0 for free, 100 for occupied, -1 for unknown)
            proposed_simplified_map_.data.resize(proposed_simplified_map_.info.width * proposed_simplified_map_.info.height, -1);

            // Map is now initialized
            initialization_flag = true;
        }
    } 

    // Define all local map callbacks
    void cyan_map_callback(const nav_msgs::msg::OccupancyGrid & msg) 
    {
        combine_map(msg);
    }
    void magenta_map_callback(const nav_msgs::msg::OccupancyGrid & msg) 
    {
        combine_map(msg);
    }
    void yellow_map_callback(const nav_msgs::msg::OccupancyGrid & msg) 
    {
        combine_map(msg);
    }
    void red_map_callback(const nav_msgs::msg::OccupancyGrid & msg) 
    {
        combine_map(msg);
    }
    void green_map_callback(const nav_msgs::msg::OccupancyGrid & msg) 
    {
        combine_map(msg);
    }
    void blue_map_callback(const nav_msgs::msg::OccupancyGrid & msg) 
    {
        combine_map(msg);
    }

    void combine_map(const nav_msgs::msg::OccupancyGrid new_map)
    {
        std::vector<std::vector<int>> proposed_simplified_grid(proposed_simplified_map_.info.width, std::vector<int>(proposed_simplified_map_.info.height, -1));

        // Look at each minimap
        const int minimap_dim = static_cast<int>(proposed_simplified_map_.info.resolution / new_map.info.resolution); // cells

        const int obstacle_threshold = 3; // 5
        const int free_threshold = 3; // 6

        for (size_t i = 0; i < proposed_simplified_map_.info.width; i++)
        {
            for (size_t j = 0; j < proposed_simplified_map_.info.height; j++)
            {
                std::vector<std::vector<int>> minimap(minimap_dim, std::vector<int>(minimap_dim, -1));

                int obstacle_count = 0; // Counter for obstacles in the minimap
                int free_count = 0; // Counter for free spaces in the minimap

                for (size_t p = 0; p < static_cast<size_t>(minimap_dim); p++)
                {
                    for (size_t q = 0; q < static_cast<size_t>(minimap_dim); q++)
                    {
                        // Calculate position of the current cell in the high-resolution map
                        double x = proposed_simplified_map_.info.origin.position.x + 
                                    (static_cast<double>(i) * proposed_simplified_map_.info.resolution) + 
                                    (static_cast<double>(p) * new_map.info.resolution);
                                    // static_cast<double>(proposed_simplified_map_.info.resolution)/2.0; // meters
                        double y = proposed_simplified_map_.info.origin.position.y + 
                                    (static_cast<double>(j) * proposed_simplified_map_.info.resolution) + 
                                    (static_cast<double>(q) * new_map.info.resolution);
                                    // static_cast<double>(proposed_simplified_map_.info.resolution)/2.0; // meters

                        // Convert position to grid indices in the high-resolution map
                        int new_map_idx_x = (x - new_map.info.origin.position.x) / new_map.info.resolution;
                        int new_map_idx_y = (y - new_map.info.origin.position.y) / new_map.info.resolution;

                        if (0 <= new_map_idx_x && new_map_idx_x < static_cast<int>(new_map.info.width) &&
                            0 <= new_map_idx_y && new_map_idx_y < static_cast<int>(new_map.info.height))
                        {
                            // Calculate index in the high-resolution map
                            int new_map_idx = new_map_idx_y * new_map.info.width + new_map_idx_x;
                            
                            minimap[p][q] = new_map.data[new_map_idx];
                            
                            // Count obstacles
                            if (minimap[p][q] == 100) {
                                obstacle_count++;
                            }
                            else if (minimap[p][q] == 0) {
                                free_count++;
                            }
                        }
                    }
                }

                // double r = 
                // double normalized_obstacle_count = 

                // Mark cells as free or obstacle. Obstacles given priority.
                if (free_count >= free_threshold) {
                    // Mark the current cell in the low-resolution map as an obstacle
                    int current_cell_idx = j * proposed_simplified_map_.info.width + i;
                    proposed_simplified_map_.data[current_cell_idx] = 0;
                }
                if (obstacle_count >= obstacle_threshold) {
                // if (obstacle_count > 0) {
                    // Mark the current cell in the low-resolution map as an obstacle
                    int current_cell_idx = j * proposed_simplified_map_.info.width + i;
                    proposed_simplified_map_.data[current_cell_idx] = 100;
                    // proposed_simplified_map_.data[current_cell_idx] = obstacle_count * 10;
                }
            }
        }
        proposed_simplified_map_publisher_->publish(proposed_simplified_map_);
    }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map_Combiner>());
  rclcpp::shutdown();
  return 0;
}









    



