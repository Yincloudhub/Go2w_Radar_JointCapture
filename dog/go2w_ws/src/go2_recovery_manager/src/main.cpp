#include <memory>
#include <functional>
#include <cstdint>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/wireless_controller.hpp"


class Go2RecoveryManagerNode : public rclcpp::Node
{
public:
    Go2RecoveryManagerNode()
        : Node("go2_recovery_manager_node")
    {
        // ============================================================
        // 订阅宇树遥控器
        // ============================================================

        subscription_ =
            this->create_subscription<unitree_go::msg::WirelessController>(
                "/wirelesscontroller",
                10,
                std::bind(
                    &Go2RecoveryManagerNode::remoteCallback,
                    this,
                    std::placeholders::_1
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "Go2恢复管理节点启动成功"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "长按X键%.1f秒，松手后重启Go2w_velocitySerial_Autostart.service",
            HOLD_TIME_SEC
        );
    }


private:

    // ============================================================
    // 参数
    // ============================================================

    // 实测：
    // X -> keys = 1024
    static constexpr uint16_t RECOVERY_KEY_MASK = 1024;

    // 长按确认时间
    static constexpr double HOLD_TIME_SEC = 2.0;


    // ============================================================
    // 状态变量
    // ============================================================

    // 当前X是否处于按下状态
    bool recovery_key_pressed_ = false;

    // 本轮长按是否已经达到2秒
    bool recovery_triggered_ = false;

    // X开始按下的时间
    std::chrono::steady_clock::time_point
        press_start_time_;


    // ============================================================
    // ROS2 Subscriber
    // ============================================================

    rclcpp::Subscription<
        unitree_go::msg::WirelessController
    >::SharedPtr subscription_;


    // ============================================================
    // 遥控器回调
    // ============================================================

    void remoteCallback(
        const unitree_go::msg::WirelessController::SharedPtr msg)
    {
        bool pressed =
            (msg->keys & RECOVERY_KEY_MASK) != 0;


        // ========================================================
        // 1.X刚刚按下
        // ========================================================

        if (pressed && !recovery_key_pressed_)
        {
            recovery_key_pressed_ = true;

            recovery_triggered_ = false;

            press_start_time_ =
                std::chrono::steady_clock::now();


            RCLCPP_INFO(
                this->get_logger(),
                "检测到恢复键X按下"
            );
        }


        // ========================================================
        // 2.X持续按住
        // ========================================================

        if (
            pressed &&
            recovery_key_pressed_ &&
            !recovery_triggered_
        )
        {
            auto now =
                std::chrono::steady_clock::now();


            std::chrono::duration<double> elapsed =
                now - press_start_time_;


            // ----------------------------------------------------
            // 达到2秒
            //
            // 注意：
            // 此时只确认，不真正重启
            // ----------------------------------------------------

            if (elapsed.count() >= HOLD_TIME_SEC)
            {
                recovery_triggered_ = true;


                RCLCPP_WARN(
                    this->get_logger(),
                    "恢复键X已长按%.1f秒，松开X后执行系统恢复",
                    HOLD_TIME_SEC
                );
            }
        }


        // ========================================================
        // 3.X松开
        // ========================================================

        if (!pressed && recovery_key_pressed_)
        {
            // 先记录是否已经满足恢复条件
            bool should_restart =
                recovery_triggered_;


            // ----------------------------------------------------
            // 先复位按键状态
            // ----------------------------------------------------

            recovery_key_pressed_ = false;
            recovery_triggered_ = false;


            // ----------------------------------------------------
            // 短按
            // ----------------------------------------------------

            if (!should_restart)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "恢复键X按下时间不足，不执行系统恢复"
                );

                return;
            }


            // ----------------------------------------------------
            // 长按达到要求，并且现在已经松手
            // ----------------------------------------------------

            RCLCPP_WARN(
                this->get_logger(),
                "检测到X松开，开始重启Go2w_velocitySerial_Autostart.service"
            );


            restartExperimentService();
        }
    }


    // ============================================================
    // 重启实验系统
    // ============================================================

    void restartExperimentService()
    {
        // --------------------------------------------------------
        // -n:
        // 禁止sudo等待输入密码
        //
        // 如果sudoers没有配置正确，会直接失败，
        // 而不是卡在这里等待密码
        // --------------------------------------------------------

        const char *command =
            "/usr/bin/sudo -n "
            "/usr/bin/systemctl restart "
            "Go2w_velocitySerial_Autostart.service";


        RCLCPP_WARN(
            this->get_logger(),
            "正在执行：systemctl restart Go2w_velocitySerial_Autostart.service"
        );


        int ret =
            std::system(command);


        if (ret == 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Go2w_velocitySerial_Autostart.service重启命令执行成功"
            );
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Go2w_velocitySerial_Autostart.service重启失败，返回值=%d",
                ret
            );
        }
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);


    auto node =
        std::make_shared<Go2RecoveryManagerNode>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}