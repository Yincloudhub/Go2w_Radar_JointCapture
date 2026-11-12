
#include <memory>
#include <functional>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <poll.h>

#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


class OdomSerialSenderNode : public rclcpp::Node
{
public:

    OdomSerialSenderNode()
        : Node("odom_serial_sender_node")
    {
        // ============================================================
        // 1. 节点启动时不主动打开串口
        // 等待B键开启串口发送
        // ============================================================

        RCLCPP_INFO(
            this->get_logger(),
            "节点启动，串口默认关闭，等待B键开启"
        );

        // ============================================================
        // 2. 订阅 Odometry
        // ============================================================

        subscription_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/unitree/slam_mapping/odom",
                10,
                std::bind(
                    &OdomSerialSenderNode::odomCallback,
                    this,
                    std::placeholders::_1
                )
            );

        // ============================================================
        // 3. 串口发送控制 Service
        // ============================================================

        odomSerialSender_service_ =
            this->create_service<std_srvs::srv::SetBool>(
                "/odom_serial_sender/set_enabled",
                std::bind(
                    &OdomSerialSenderNode::setEnabledCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );

        // ============================================================
        // 4. 串口状态检测及自动重连定时器
        // 每隔2秒执行一次
        // ============================================================

        serial_timer_ =
            this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(
                    &OdomSerialSenderNode::serialMaintenanceCallback,
                    this
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "节点启动，开始订阅 /unitree/slam_mapping/odom"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "串口自动重连机制已就绪，检测周期：2秒"
        );
    }


    ~OdomSerialSenderNode()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;

            RCLCPP_INFO(
                this->get_logger(),
                "节点退出，串口已关闭"
            );
        }
    }


private:

    // ============================================================
    // 1. 串口参数
    // ============================================================

    const std::string serial_port_ = "/dev/Ch340_odom";

    const speed_t baudrate_ = B115200;

    int serial_fd_ = -1;

    // 用户是否允许发送Odometry
    bool send_enabled_ = false;


    // ============================================================
    // 2. ROS 2 对象
    // ============================================================

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
        odomSerialSender_service_;

    rclcpp::TimerBase::SharedPtr serial_timer_;


    // ============================================================
    // 3. 关闭串口
    // ============================================================

    void closeSerial(const std::string &reason)
    {
        if (serial_fd_ >= 0)
        {
            int fd = serial_fd_;

            // 先将文件描述符置为无效
            serial_fd_ = -1;

            close(fd);

            RCLCPP_WARN(
                this->get_logger(),
                "串口已关闭：%s",
                reason.c_str()
            );
        }
    }


    // ============================================================
    // 4. 串口初始化
    // ============================================================

    bool initSerial(const std::string &port, speed_t baudrate)
    {
        // 防止重复打开
        if (serial_fd_ >= 0)
        {
            return true;
        }

        serial_fd_ = open(
            port.c_str(),
            O_RDWR | O_NOCTTY
        );

        if (serial_fd_ < 0)
        {
            int err = errno;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "无法打开串口 %s：%s",
                port.c_str(),
                std::strerror(err)
            );

            return false;
        }

        struct termios tty{};

        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            int err = errno;

            close(serial_fd_);
            serial_fd_ = -1;

            RCLCPP_ERROR(
                this->get_logger(),
                "读取串口配置失败：%s",
                std::strerror(err)
            );

            return false;
        }

        cfmakeraw(&tty);

        cfsetospeed(&tty, baudrate);
        cfsetispeed(&tty, baudrate);

        tty.c_cflag =
            (tty.c_cflag & ~CSIZE) | CS8;

        tty.c_iflag &= ~IGNBRK;

        tty.c_lflag = 0;
        tty.c_oflag = 0;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        tty.c_iflag &=
            ~(IXON | IXOFF | IXANY);

        tty.c_cflag |=
            (CLOCAL | CREAD);

        tty.c_cflag &=
            ~(PARENB | PARODD);

        tty.c_cflag &=
            ~CSTOPB;

        tty.c_cflag &=
            ~CRTSCTS;

        if (tcsetattr(
                serial_fd_,
                TCSANOW,
                &tty) != 0)
        {
            int err = errno;

            close(serial_fd_);
            serial_fd_ = -1;

            RCLCPP_ERROR(
                this->get_logger(),
                "串口参数配置失败：%s",
                std::strerror(err)
            );

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "串口打开成功：%s，波特率：115200",
            port.c_str()
        );

        return true;
    }


    // ============================================================
    // 5. 串口状态检测及自动重连
    // ============================================================

    void serialMaintenanceCallback()
    {
        // --------------------------------------------------------
        // 情况1：用户主动关闭了串口发送
        // 不进行任何重连操作
        // --------------------------------------------------------

        if (!send_enabled_)
        {
            // 防止出现状态不一致
            if (serial_fd_ >= 0)
            {
                closeSerial("串口发送功能未使能");
            }

            return;
        }


        // --------------------------------------------------------
        // 情况2：串口未打开，尝试重新连接
        // --------------------------------------------------------

        if (serial_fd_ < 0)
        {
            if (initSerial(serial_port_, baudrate_))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "串口重新连接成功：%s",
                    serial_port_.c_str()
                );
            }

            return;
        }


        // --------------------------------------------------------
        // 情况3：串口已经打开，检测设备是否断开
        // --------------------------------------------------------

        struct pollfd pfd{};

        pfd.fd = serial_fd_;

        // 只检测异常事件
        pfd.events = 0;

        int ret = poll(&pfd, 1, 0);

        if (ret < 0)
        {
            if (errno == EINTR)
            {
                return;
            }

            int err = errno;

            RCLCPP_ERROR(
                this->get_logger(),
                "串口状态检测失败：%s",
                std::strerror(err)
            );

            closeSerial("串口状态检测异常");

            return;
        }


        // --------------------------------------------------------
        // 检测到USB串口异常或设备断开
        // --------------------------------------------------------

        if (ret > 0 &&
            (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "检测到串口异常，事件标志：0x%x",
                pfd.revents
            );

            closeSerial("检测到USB串口断开");

            // send_enabled_保持true
            // 后续定时器会自动重新连接
        }
    }


    // ============================================================
    // 6. Service：控制串口发送使能
    // ============================================================

    void setEnabledCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // --------------------------------------------------------
        // B键开启
        // --------------------------------------------------------

        if (request->data)
        {
            send_enabled_ = true;

            // 立即尝试打开串口
            // 如果失败，交给2秒定时器继续重连
            if (serial_fd_ < 0)
            {
                initSerial(serial_port_, baudrate_);
            }

            response->success = true;

            if (serial_fd_ >= 0)
            {
                response->message =
                    "Odometry串口发送已开启";
            }
            else
            {
                response->message =
                    "发送已使能，串口未连接，等待自动重连";
            }

            RCLCPP_INFO(
                this->get_logger(),
                "%s",
                response->message.c_str()
            );
        }

        // --------------------------------------------------------
        // B键关闭
        // --------------------------------------------------------

        else
        {
            send_enabled_ = false;

            // 主动关闭串口，释放USB设备
            closeSerial("用户主动关闭串口发送");

            response->success = true;

            response->message =
                "Odometry串口发送已停止，串口已释放";

            RCLCPP_INFO(
                this->get_logger(),
                "%s",
                response->message.c_str()
            );
        }
    }


    // ============================================================
    // 7. 串口完整数据发送
    // ============================================================

    bool writeSerialData(const std::string &data)
    {
        if (serial_fd_ < 0)
        {
            return false;
        }

        size_t total_written = 0;

        while (total_written < data.size())
        {
            ssize_t written = write(
                serial_fd_,
                data.data() + total_written,
                data.size() - total_written
            );

            if (written > 0)
            {
                total_written +=
                    static_cast<size_t>(written);

                continue;
            }

            // 被信号中断，继续发送
            if (written < 0 && errno == EINTR)
            {
                continue;
            }

            int err = (written == 0) ? EIO : errno;

            RCLCPP_ERROR(
                this->get_logger(),
                "串口发送失败：%s",
                std::strerror(err)
            );

            // 发送失败，关闭当前串口
            // send_enabled_保持true
            // 后续由定时器自动重新连接
            closeSerial("串口发送失败，等待自动重连");

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "串口发送%zu字节：%s",
            total_written,
            data.c_str()
        );

        return true;
    }


    // ============================================================
    // 8. Odometry 回调
    // ============================================================

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 用户没有开启发送
        if (!send_enabled_)
        {
            return;
        }

        // 串口尚未连接
        if (serial_fd_ < 0)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Odometry已收到，但串口未连接，等待重连"
            );

            return;
        }


        // --------------------------------------------------------
        // 组织Odometry数据帧
        // --------------------------------------------------------

        std::ostringstream oss;

        oss << "ODOM" << ","
            << msg->header.stamp.sec << ","
            << msg->header.stamp.nanosec << ","

            << std::fixed
            << std::setprecision(6)

            << msg->pose.pose.position.x << ","
            << msg->pose.pose.position.y << ","
            << msg->pose.pose.position.z << ","

            << msg->pose.pose.orientation.x << ","
            << msg->pose.pose.orientation.y << ","
            << msg->pose.pose.orientation.z << ","
            << msg->pose.pose.orientation.w << ","

            << msg->twist.twist.linear.x << ","
            << msg->twist.twist.linear.y << ","
            << msg->twist.twist.linear.z << ","

            << msg->twist.twist.angular.x << ","
            << msg->twist.twist.angular.y << ","
            << msg->twist.twist.angular.z

            << "\r\n";


        std::string send_data = oss.str();

        // 发送串口数据
        writeSerialData(send_data);
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<OdomSerialSenderNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
