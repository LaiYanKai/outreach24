#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "capture/capture.hpp"

namespace outreach24
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class Capture : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;

        // // ----------- Timers -------------------
        rclcpp::TimerBase::SharedPtr timer_main; // contains the timer that runs the main looping function at regular intervals.

        // ----------- Parameters ---------------
        std::string topic;

        // ----------- States -------------
        bool need_capture;

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Capture()
            : Node("capture")
        {
            initStates();
            initParams();
            initTopics();
            initTimers();
        }

    private:
        void initStates()
        {
            need_capture = false;
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            topic = "/camera/image_raw"; // default value
            initParam(this, "topic", topic);
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            sub_img = this->create_subscription<sensor_msgs::msg::Image>(
                topic, rclcpp::SensorDataQoS(),
                std::bind(&Capture::cbImg, this, std::placeholders::_1));
        }

        void cbImg(sensor_msgs::msg::Image::SharedPtr msg)
        {
            if (need_capture)
                msg->data;
        }

        /** Initializes the timers with their callbacks.*/
        void initTimers()
        {
            timer_main = this->create_wall_timer(
                0.1s,
                std::bind(&Capture::cbTimer, this));
        }

        /** The function that is run at regular intervals */
        void cbTimer()
        {
            
            
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<outreach24::Capture>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}