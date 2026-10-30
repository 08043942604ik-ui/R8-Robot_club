#include "Publish_realsense.h"


        RealSenseColorNode::RealSenseColorNode()
        : Node("realsense_color_node")
    {
        publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                "/camera/color/image_raw",
                10
            );

        rs2::config config;

        config.enable_stream(
            RS2_STREAM_COLOR,
            640,
            480,
            RS2_FORMAT_RGB8,
            15
        );
      
        rs2::context ctx;
auto devices = ctx.query_devices();

RCLCPP_INFO(
    this->get_logger(),
    "RealSense device count = %zu",
    devices.size()
);

started_ = false;

color_enabled_.store(
    declare_parameter<bool>("enable_color", false)
);

parameter_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // 先に型を確認
        for (const auto& parameter : parameters) {
            if (parameter.get_name() == "enable_color" &&
                parameter.get_type() !=
                    rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = "enable_color must be bool";
                return result;
            }
        }

        for (const auto& parameter : parameters) {
            if (parameter.get_name() == "enable_color") {
                color_enabled_.store(parameter.as_bool());
            }
        }

        return result;
    }
);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RealSenseColorNode::publish_frame, this)
        );

        RCLCPP_INFO(this->get_logger(), "RealSense color node started");
    }

        RealSenseColorNode::~RealSenseColorNode()
    {
        if (started_)
        {

                 RCLCPP_INFO(this->get_logger(), "Stopping RealSense pipeline...");
            try
            {

                pipeline_.stop();
            }
            catch (const rs2::error& de)
            {
                            RCLCPP_ERROR(
                this->get_logger(),
                "RealSense error: %s",
                de.what()
            );
            }
        
        RCLCPP_INFO(this->get_logger(), "RealSense pipeline stopped");
        }
    }

void RealSenseColorNode::publish_frame()
    {
        try
        {

            const bool enabled = color_enabled_.load();

if (!enabled) {
    if (started_) {
        pipeline_.stop();
        started_ = false;
        RCLCPP_INFO(get_logger(), "RealSense color OFF");
    }

    return;
}

if (!started_) {
    rs2::config config;

    config.enable_stream(
        RS2_STREAM_COLOR,
        640,
        480,
        RS2_FORMAT_RGB8,
        15
    );

    pipeline_.start(config);
    started_ = true;

    RCLCPP_INFO(get_logger(), "RealSense color ON");
}

            rs2::frameset frames;

            if (!pipeline_.poll_for_frames(&frames))
                return;

            auto color = frames.get_color_frame();

            if (!color)
                return;

            sensor_msgs::msg::Image msg;

            msg.header.stamp = this->now();
            msg.header.frame_id = "camera_color_optical_frame";

            msg.height = color.get_height();
            msg.width = color.get_width();
            msg.encoding = "rgb8";
            msg.is_bigendian = false;
            msg.step = msg.width * 3;

            const size_t size = msg.step * msg.height;

            msg.data.resize(size);

            std::memcpy(
                msg.data.data(),
                color.get_data(),
                size
            );

            publisher_->publish(msg);
        }
        catch (const rs2::error &e  )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "RealSense error: %s",
                e.what()
            );
        }
    }



