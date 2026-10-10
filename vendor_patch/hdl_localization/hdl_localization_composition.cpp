#include <memory>
#include "./hdl_localization_app.hpp"
#include "./globalmap_server_app.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto hdl_localization_localization = std::make_shared<hdl_localization::HdlLocalizationApp>(options);
    auto global_map_server = std::make_shared<hdl_localization::GlobalmapServerApp>(options);
    exec.add_node(hdl_localization_localization);
    exec.add_node(global_map_server);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
