#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"

#include "capture/capture.hpp"

namespace outreach24
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class Capture : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
        sensor_msgs::msg::Image::SharedPtr img_;

        // // ----------- Timers -------------------
        rclcpp::TimerBase::SharedPtr timer_main; // contains the timer that runs the main looping function at regular intervals.

        // ----------- Parameters ---------------
        std::string topic;
        std::string img_prefix;
        std::string img_ext;
        int num_zeros;
        double period;

        // ----------- States -------------
        bool need_capture;
        bool has_img;
        int i;

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
            has_img = false;
            i = 0;
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            topic = "/camera/image_raw"; // default value
            initParam(this, "topic", topic);
            img_prefix = "img";
            initParam(this, "img_prefix", img_prefix);
            img_ext = ".jpg";
            initParam(this, "img_ext", img_ext);
            num_zeros = 4;
            initParam(this, "num_zeros", num_zeros);
            period = 1;
            initParam(this, "period", period);
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            auto qos = rclcpp::ServicesQoS();
            // qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            qos.keep_last(1);
            sub_img = this->create_subscription<sensor_msgs::msg::Image>(
                topic, qos,
                std::bind(&Capture::cbImg, this, std::placeholders::_1));
        }

        void cbImg(sensor_msgs::msg::Image::SharedPtr msg)
        {
            img_ = msg;
            has_img = true;
        }

        /** Initializes the timers with their callbacks.*/
        void initTimers()
        {
            timer_main = this->create_wall_timer(
                1s * period,
                std::bind(&Capture::cbTimer, this));
        }

        /** The function that is run at regular intervals */
        void cbTimer()
        {
            // char c;

            // if (getch(c) == true || rclcpp::ok() == false)
            // {
            //     // std::cout << std::endl;
            //     timer_main = nullptr;
            //     return;
            // }

            // switch (c)
            // {
            // case 'C':
            // case ' ':
            // case 'c': // forward
            //     // std::cout << "to capture" << std::endl;
            //     need_capture = !need_capture;
            //     break;

            // default: // don't do anything.
            //     break;
            // }
            // std::cout << "ca" << need_capture<< has_img << std::endl;

            if (has_img){
                cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(img_);
                std::ostringstream ss;
                ss << "img/" << img_prefix << std::setw(num_zeros) << std::setfill('0') << i << "." << img_ext;
                std::string fname = ss.str();
                ++i;
                
                cv::imwrite(fname, img->image);

                std::cout << fname << std::endl;
            }
            // std::cout << "\r[" << c << "] ";
            // std::cout << std::fixed;
            // std::cout << " LinVel("
            //           << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.linear.x << ", "
            //           << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.linear.y << ", "
            //           << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.linear.z << ")";
            // std::cout << " YawVel("
            //           << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.angular.z << ")";
            // std::cout << " Steps(Lin:"
            //           << std::setw(5) << std::setprecision(2) << params_.linear_step << ", Ang"
            //           << std::setw(5) << std::setprecision(2) << params_.angular_step << ")";

            // std::cout << "    "; // pad some spaces just in case
            // std::cout.flush();
            // std::cout << "\b\b\b\b"; // remove extra nonsense characters.
            // std::cout.flush();
        }

        bool getch(char &c)
        {
            c = 0;
            termios old;
            bool error = false;
            if (tcgetattr(0, &old) < 0)
                return true; // perror("tcsetattr()");
            old.c_lflag &= ~ICANON;
            old.c_lflag &= ~ECHO;
            old.c_cc[VMIN] = 1;
            old.c_cc[VTIME] = 0;
            if (tcsetattr(0, TCSANOW, &old) < 0)
                error = true; // perror("tcsetattr ICANON");
            if (read(0, &c, 1) < 0)
                error = true;
            old.c_lflag |= ICANON;
            old.c_lflag |= ECHO;
            if (tcsetattr(0, TCSADRAIN, &old) < 0)
                error = true; // perror("tcsetattr ~ICANON");
            return error;
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