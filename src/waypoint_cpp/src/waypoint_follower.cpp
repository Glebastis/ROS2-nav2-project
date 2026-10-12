#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <map>
#include <functional>
#include <chrono>
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::placeholders;

class WaypointFollower : public rclcpp::Node
{
public:
    WaypointFollower() : Node("waypoint_follower")
    {
        this->declare_parameter<std::vector<double>>(
            "waypoints",
            {0.0, 0.0, 2.5, -1.5, 5.0, 1.5, 10.0, 0.0});
        this->declare_parameter<int>("max_retries", 2);
        this->declare_parameter<double>("retry_delay_s", 2.0);

        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        waypoints_     = this->get_parameter("waypoints").as_double_array();
        max_retries_   = this->get_parameter("max_retries").as_int();
        retry_delay_s_ = this->get_parameter("retry_delay_s").as_double();

        RCLCPP_INFO(this->get_logger(),
            "Waypoint follower started, %zu waypoints, max_retries=%d, delay=%.1fs",
            waypoints_.size() / 2, max_retries_, retry_delay_s_);

        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(30))) {
            RCLCPP_ERROR(this->get_logger(),
                "navigate_to_pose action server not available after 30s");
            rclcpp::shutdown();
            return;
        }

        send_next(0);
    }

private:
    void send_next(int i)
    {
        if (i * 2 >= static_cast<int>(waypoints_.size())) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached");
            rclcpp::shutdown();
            return;
        }

        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->get_clock()->now();
        goal.pose.pose.position.x = waypoints_[i * 2];
        goal.pose.pose.position.y = waypoints_[i * 2 + 1];
        goal.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(),
            "Sending waypoint %d: (%f, %f)",
            i, waypoints_[i * 2], waypoints_[i * 2 + 1]);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback =
            std::bind(&WaypointFollower::goal_response_callback, this, _1, i);
        options.result_callback =
            std::bind(&WaypointFollower::goal_result_callback, this, _1, i);

        navigate_to_pose_client_->async_send_goal(goal, options);
    }

    void schedule_retry(int i, const std::string &reason)
    {
        int attempt = ++retries_[i];
        if (attempt > max_retries_) {
            RCLCPP_ERROR(this->get_logger(),
                "Waypoint %d failed %d times (%s), skipping",
                i, max_retries_, reason.c_str());
            send_next(i + 1);
            return;
        }

        RCLCPP_WARN(this->get_logger(),
            "Waypoint %d %s, retry %d/%d in %.1fs",
            i, reason.c_str(), attempt, max_retries_, retry_delay_s_);

        auto timer = this->create_wall_timer(
            std::chrono::duration<double>(retry_delay_s_),
            [this, i]() {
                auto it = retry_timers_.find(i);
                if (it != retry_timers_.end() && it->second) {
                    it->second->cancel();
                    retry_timers_.erase(it);
                }
                send_next(i);
            });
        retry_timers_[i] = timer;
    }

    void goal_response_callback(
        const NavigateToPoseGoalHandle::SharedPtr &goal_handle,
        int i)
    {
        if (!goal_handle) {
            schedule_retry(i, "rejected");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waypoint %d accepted", i);
    }

    void goal_result_callback(
        const NavigateToPoseGoalHandle::WrappedResult &result,
        int i)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Waypoint %d SUCCEEDED", i);
                send_next(i + 1);
                break;

            case rclcpp_action::ResultCode::ABORTED:
                schedule_retry(i, "ABORTED");
                break;

            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Waypoint %d CANCELED", i);
                send_next(i + 1);
                break;

            default:
                RCLCPP_INFO(this->get_logger(),
                    "Waypoint %d finished with unknown status %d",
                    i, static_cast<int>(result.code));
                send_next(i + 1);
                break;
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    std::vector<double> waypoints_;
    std::map<int, int> retries_;
    std::map<int, rclcpp::TimerBase::SharedPtr> retry_timers_;
    int    max_retries_   = 2;
    double retry_delay_s_ = 2.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}