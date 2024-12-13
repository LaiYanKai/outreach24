#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace outreach24
{
    template <typename T>
    void initParam(rclcpp::Node *const &node, const std::string &param_name, T &to, const size_t &cout_width = 25)
    {
        if (node->has_parameter(param_name) == false)
            node->declare_parameter<T>(param_name, to);
        node->get_parameter_or<T>(param_name, to, to);

        if (cout_width > 0)
            std::cout << std::setw(cout_width) << param_name << ": " << to << std::endl;
    }
}