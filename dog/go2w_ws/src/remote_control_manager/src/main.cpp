#include <memory>
#include <functional>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "std_srvs/srv/set_bool.hpp"

class RemoteControlManagerNode : public rclcpp::Node
{
public:
    RemoteControlManagerNode() : Node("remote_control_manager_node")
    {
        // ============================================================
        // 订阅宇树遥控器
        // ============================================================
        subscription_ = this->create_subscription<unitree_go::msg::WirelessController>("/wirelesscontroller",10,
                std::bind(
                    &RemoteControlManagerNode::remoteCallback,
                    this,
                    std::placeholders::_1
                )
            );
        RCLCPP_INFO(
            this->get_logger(),
            "遥控器管理节点启动成功"
        );
        RCLCPP_INFO(
            this->get_logger(),
            "按键A：相机录像控制"
        );
        RCLCPP_INFO(
            this->get_logger(),
            "按键B：Odometry串口发送控制"
        );
        camera_client_ =this->create_client<std_srvs::srv::SetBool>("/camera_remote_recorder/set_recording");
        serial_client_ =this->create_client<std_srvs::srv::SetBool>("/odom_serial_sender/set_enabled");
    }

private:
    // ============================================================
    // 遥控器按键掩码
    //
    // 实测：
    // A -> keys = 256
    // B -> keys = 512
    // ============================================================

    static constexpr uint16_t A_MASK = 256;
    static constexpr uint16_t B_MASK = 512;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr camera_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr serial_client_;

    // ============================================================
    // 上一次按键状态
    //
    // 用于检测：
    // 0 -> 1
    // 即按键的上升沿
    // ============================================================

    bool a_last_pressed_ = false;
    bool b_last_pressed_ = false;
    //============================================================
    bool recording_enabled_ = false;
    bool serial_enabled_ = false;
    // 串口Service请求是否正在处理中
    bool serial_request_pending_ = false;
    // ============================================================
    // ROS2 Subscriber
    // ============================================================

    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr subscription_;


    // ============================================================
    // 遥控器回调
    // ============================================================

    void remoteCallback(const unitree_go::msg::WirelessController::SharedPtr msg)
    {
        // --------------------------------------------------------
        // 当前A状态
        // --------------------------------------------------------
        bool a_pressed =
            (msg->keys & A_MASK) != 0;
        // --------------------------------------------------------
        // 当前B状态
        // --------------------------------------------------------

        bool b_pressed =
            (msg->keys & B_MASK) != 0;


        // ========================================================
        // A上升沿
        // 上一帧：没有按
        // 当前帧：按下
        // ========================================================

        if (a_pressed && !a_last_pressed_)
        {
            RCLCPP_INFO(this->get_logger(),"检测到A按下");
          
            if (!camera_client_->service_is_ready()) 
                RCLCPP_ERROR(this->get_logger(), "相机录像Service当前不可用");
            else
            {
                bool target_state = !recording_enabled_;
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = target_state;
                camera_client_->async_send_request(request,
                    [this, target_state](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
                    {
                        auto response = future.get();
                        if (response->success)
                            recording_enabled_ = target_state;  //更新本地状态
                    }
                );
            }
        }


        // ========================================================
        // B上升沿
        // ========================================================
        if (b_pressed && !b_last_pressed_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "检测到B按下"
            );

            // 防止上一个Service请求尚未完成
            if (serial_request_pending_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "串口控制请求正在处理中，请稍后"
                );
            }
            else if (!serial_client_->service_is_ready())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Odometry串口发送Service当前不可用"
                );
            }
            else
            {
                bool target_state = !serial_enabled_;

                auto request =
                    std::make_shared<std_srvs::srv::SetBool::Request>();

                request->data = target_state;

                serial_request_pending_ = true;

                serial_client_->async_send_request(
                    request,
                    [this, target_state](
                        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
                    {
                        serial_request_pending_ = false;

                        try
                        {
                            auto response = future.get();

                            if (response->success)
                            {
                                serial_enabled_ = target_state;

                                RCLCPP_INFO(
                                    this->get_logger(),
                                    "Odometry串口发送状态：%s",
                                    serial_enabled_ ? "开启" : "关闭"
                                );
                            }
                            else
                            {
                                RCLCPP_ERROR(
                                    this->get_logger(),
                                    "串口控制失败：%s",
                                    response->message.c_str()
                                );
                            }
                        }
                        catch (const std::exception &e)
                        {
                            RCLCPP_ERROR(
                                this->get_logger(),
                                "串口控制Service请求异常：%s",
                                e.what()
                            );
                        }
                    }
                );
            }
        }


        // ========================================================
        // 保存当前状态
        //
        // 留给下一帧比较
        // ========================================================

        a_last_pressed_ = a_pressed;
        b_last_pressed_ = b_pressed;
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<RemoteControlManagerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}