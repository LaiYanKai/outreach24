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

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Capture()
            : Node("capture")
        {
            initParams();
            initTopics();
            initTimers();
        }

    private:
        /** Initializes and read parameters, if any. */
        void initParams()
        {
            // initParam(this, "cost_exponent", cost_exponent);
            // initParam(this, "circumscribed_radius", circumscribed_radius);
            // initParam(this, "inflation_radius", inflation_radius);
            // initParam(this, "max_cost", max_cost);
            // initParam(this, "min_cost", min_cost);
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
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
            std::cout << "hi " <<std::endl;
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