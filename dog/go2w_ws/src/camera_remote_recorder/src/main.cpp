#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree_api/msg/request.hpp"


class CameraRemoteRecorderNode : public rclcpp::Node
{
public:
    CameraRemoteRecorderNode():Node("camera_remote_recorder_node")
    {
        // ============================================================
        // 1.打开摄像头
        // ============================================================
        if (!camera_.open(CAMERA_DEVICE, cv::CAP_V4L2))
        {
            throw std::runtime_error(
                std::string("无法打开摄像头：") + CAMERA_DEVICE);
        }

        // 尝试设置摄像头参数

        camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G')); // 优先让USB摄像头使用MJPEG输出，降低USB带宽压力
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
        camera_.set(cv::CAP_PROP_FPS, CAMERA_FPS);
        
        
        

        // 获取摄像头实际参数
        actual_width_ =
            static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH));

        actual_height_ =
            static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT));

        actual_fps_ = camera_.get(cv::CAP_PROP_FPS);

        if (actual_fps_ <= 1.0)
        {
            actual_fps_ = CAMERA_FPS;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "摄像头初始化成功：%s",
            CAMERA_DEVICE);

        RCLCPP_INFO(
            this->get_logger(),
            "摄像头实际参数：%dx%d @ %.2f FPS",
            actual_width_,
            actual_height_,
            actual_fps_);

        // ============================================================
        // 2.创建视频保存目录
        // ============================================================
        try
        {
            std::filesystem::create_directories(videoSaveDir());
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error(
                std::string("无法创建录像目录：") + e.what());
        }

        RCLCPP_INFO(
            this->get_logger(),
            "录像保存目录：%s",
            videoSaveDir().c_str());

        // ============================================================
        // 3.AudioHub / VUI Publisher
        // ============================================================
        audiohub_pub_ =
            this->create_publisher<unitree_api::msg::Request>(
                "/api/audiohub/request",
                10);

        vui_pub_ =
            this->create_publisher<unitree_api::msg::Request>(
                "/api/vui/request",
                10);

        // ============================================================
        // 4.录像控制Service
        // ============================================================
        recording_service_ =
            this->create_service<std_srvs::srv::SetBool>(
                "/camera_remote_recorder/set_recording",
                std::bind(
                    &CameraRemoteRecorderNode::recordingServiceCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        // ============================================================
        // 5.摄像头采集Timer
        // ============================================================
        int timer_period_ms =
            static_cast<int>(1000.0 / actual_fps_);

        if (timer_period_ms < 1)
        {
            timer_period_ms = 33;
        }

        camera_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(timer_period_ms),
                std::bind(
                    &CameraRemoteRecorderNode::cameraTimerCallback,
                    this));

        // ============================================================
        // 6.设置机器狗语音音量
        // ============================================================
        setVoiceVolume(VOICE_VOLUME);

        RCLCPP_INFO(
            this->get_logger(),
            "camera_remote_recorder_node启动成功");

        RCLCPP_INFO(
            this->get_logger(),
            "当前状态：等待录像命令");
    }


    ~CameraRemoteRecorderNode()
    {
        stopRecording(false);

        if (camera_.isOpened())
        {
            camera_.release();
        }

        RCLCPP_INFO(
            this->get_logger(),
            "摄像头资源已释放");
    }


private:

    // ============================================================
    // 调试参数
    // ============================================================

    static constexpr const char *CAMERA_DEVICE =
    "/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0";

    static std::string videoSaveDir()
    {
        const char *root = std::getenv("NX_FLEET_ROOT");
        if (root && *root)
            return std::string(root) + "/data/video";
        const char *home = std::getenv("HOME");
        return std::string(home && *home ? home : "/home/unitree") + "/NX_radar_fleet/data/video";
    }

    static constexpr int CAMERA_WIDTH = 1920;
    static constexpr int CAMERA_HEIGHT = 1080;
    static constexpr double CAMERA_FPS = 30.0;

    // Go2 VUI音量范围0~10
    static constexpr int VOICE_VOLUME = 10;

    // 当前暂时使用Go2内置提示音
    static constexpr int START_RECORD_AUDIO_API = 3001;
    static constexpr int STOP_RECORD_AUDIO_API = 3002;


    // ============================================================
    // 成员变量
    // ============================================================

    cv::VideoCapture camera_;
    cv::VideoWriter video_writer_;

    bool recording_ = false;

    int actual_width_ = 0;
    int actual_height_ = 0;
    double actual_fps_ = 30.0;

    std::string current_video_path_;

    rclcpp::TimerBase::SharedPtr camera_timer_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
        recording_service_;

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr
        audiohub_pub_;

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr
        vui_pub_;


    // ============================================================
    // 摄像头采集
    // ============================================================

    void cameraTimerCallback()
    {
        cv::Mat frame;

        if (!camera_.read(frame))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "摄像头读取失败");

            return;
        }

        if (frame.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "摄像头返回空图像");

            return;
        }

        // 没录像时，只持续读取摄像头，不写硬盘
        if (!recording_)
        {
            return;
        }

        if (!video_writer_.isOpened())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "录像状态为ON，但是VideoWriter没有打开");

            recording_ = false;
            return;
        }

        video_writer_.write(frame);
    }


    // ============================================================
    // Service控制
    // ============================================================

    void recordingServiceCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            // ========================================================
            // 开始录像
            // ========================================================

            if (recording_)
            {
                response->success = true;
                response->message = "当前已经处于录像状态";

                RCLCPP_INFO(
                    this->get_logger(),
                    "收到开始录像命令，但当前已经在录像");

                return;
            }

            if (startRecording())
            {
                response->success = true;
                response->message = "录像启动成功";
            }
            else
            {
                response->success = false;
                response->message = "录像启动失败";
            }
        }
        else
        {
            // ========================================================
            // 停止录像
            // ========================================================

            if (!recording_)
            {
                response->success = true;
                response->message = "当前没有录像";

                RCLCPP_INFO(
                    this->get_logger(),
                    "收到停止录像命令，但当前没有录像");

                return;
            }

            stopRecording(true);

            response->success = true;
            response->message = "录像停止成功";
        }
    }


    // ============================================================
    // 开始录像
    // ============================================================

    bool startRecording()
    {
        if (!camera_.isOpened())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "摄像头没有打开");

            return false;
        }

        current_video_path_ = generateVideoFileName();

        int codec =cv::VideoWriter::fourcc('m','p','4','v');

        cv::Size frame_size(
            actual_width_,
            actual_height_);

        bool opened = video_writer_.open(current_video_path_,codec,actual_fps_,frame_size,true);

        if (!opened)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "无法创建录像文件：%s",
                current_video_path_.c_str());

            return false;
        }

        recording_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "开始录像");

        RCLCPP_INFO(
            this->get_logger(),
            "文件：%s",
            current_video_path_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "========================================");

        // 确认录像真正启动成功后再播放提示音
        playAudioHubPrompt(
            START_RECORD_AUDIO_API);

        return true;
    }


    // ============================================================
    // 停止录像
    // ============================================================

    void stopRecording(bool play_voice)
    {
        if (!recording_)
        {
            return;
        }

        recording_ = false;

        if (video_writer_.isOpened())
        {
            video_writer_.release();
        }

        RCLCPP_INFO(
            this->get_logger(),
            "========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "录像已停止");

        RCLCPP_INFO(
            this->get_logger(),
            "视频已保存：%s",
            current_video_path_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "========================================");

        if (play_voice)
        {
            playAudioHubPrompt(
                STOP_RECORD_AUDIO_API);
        }

        current_video_path_.clear();
    }


    // ============================================================
    // 自动生成视频文件名
    //
    // 例如：
    // NX_FLEET_ROOT/data/video/record_20260828_170501.mp4
    // ============================================================

    std::string generateVideoFileName()
    {
        auto now =
            std::chrono::system_clock::now();

        std::time_t now_time =
            std::chrono::system_clock::to_time_t(now);

        std::tm local_tm{};

        localtime_r(
            &now_time,
            &local_tm);

        char time_buffer[64];

        std::strftime(
            time_buffer,
            sizeof(time_buffer),
            "%Y%m%d_%H%M%S",
            &local_tm);

        std::string file_path =
            videoSaveDir()
            + "/record_"
            + time_buffer
            + ".mp4";

        return file_path;
    }


    // ============================================================
    // 设置Go2 VUI音量
    //
    // API 1003:
    // {"volume":6}
    // ============================================================

    void setVoiceVolume(int volume)
    {
        if (volume < 0)
        {
            volume = 0;
        }

        if (volume > 10)
        {
            volume = 10;
        }

        unitree_api::msg::Request msg;

        msg.header.identity.api_id = 1003;

        msg.parameter =
            "{\"volume\":"
            + std::to_string(volume)
            + "}";

        msg.binary.clear();

        vui_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "设置机器狗语音音量：%d",
            volume);
    }


    // ============================================================
    // 播放Go2 AudioHub内置提示音
    // ============================================================

    void playAudioHubPrompt(int api_id)
    {
        unitree_api::msg::Request msg;

        msg.header.identity.api_id =
            api_id;

        msg.parameter = "{}";
        msg.binary.clear();

        audiohub_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "发送AudioHub提示音请求，api_id=%d",
            api_id);
    }
};
   

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    try
    {
        auto node =
            std::make_shared<CameraRemoteRecorderNode>();

        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(
                "camera_remote_recorder"),
            "节点启动失败：%s",
            e.what());
    }

    rclcpp::shutdown();

    return 0;
}
