#ifndef UBOARD_SYNC_NODE_HPP
#define UBOARD_SYNC_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "board_detector.hpp"
#include "agent_manager.hpp"
#include "udev_manager.hpp"
#include "config_loader.hpp"

class UBoardSyncNode : public rclcpp::Node
{
public:
    UBoardSyncNode();

private:
    void initialize();
    void handle_board_events();

    std::shared_ptr<BoardDetector> board_detector_;
    std::shared_ptr<AgentManager> agent_manager_;
    std::shared_ptr<UdevManager> udev_manager_;
    std::shared_ptr<ConfigLoader> config_loader_;
};

#endif // UBOARD_SYNC_NODE_HPP