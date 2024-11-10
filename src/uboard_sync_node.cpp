#include "uboard-sync/uboard_sync_node.hpp"

UBoardSyncNode::UBoardSyncNode()
    : Node("uboard_sync_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting uBoardSync Node...");
    initialize();
}

void UBoardSyncNode::initialize()
{
    config_loader_ = std::make_shared<ConfigLoader>("boards_config.yaml");
    udev_manager_ = std::make_shared<UdevManager>();
    board_detector_ = std::make_shared<BoardDetector>();
    agent_manager_ = std::make_shared<AgentManager>();

    // Set up board detection callback
    board_detector_->set_board_connected_callback(
        [this](const std::string &board_name) {
            RCLCPP_INFO(this->get_logger(), "Board connected: %s", board_name.c_str());
            agent_manager_->start_agent_for_board(board_name);
        });

    board_detector_->set_board_disconnected_callback(
        [this](const std::string &board_name) {
            RCLCPP_INFO(this->get_logger(), "Board disconnected: %s", board_name.c_str());
            agent_manager_->stop_agent_for_board(board_name);
        });

    handle_board_events();
}

void UBoardSyncNode::handle_board_events()
{
    // Start monitoring for board events
    board_detector_->start_monitoring();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UBoardSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
