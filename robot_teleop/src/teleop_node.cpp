#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class TeleopNode : public rclcpp::Node
{
public:

    TeleopNode()
    : Node("teleop_node")
    {
        landmark_sub_ =
            this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/hand/landmarks",
                10,
                std::bind(&TeleopNode::landmarkCallback, this, std::placeholders::_1)
            );

        joint_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states",
                10
            );

        previous_joints_.assign(15, 0.0);

        RCLCPP_INFO(this->get_logger(), "Teleop Node Started");
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr landmark_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    std::vector<double> previous_joints_;
    double alpha_ = 0.7;

    std::vector<double> vec(const std::vector<double>& a, const std::vector<double>& b)
    {
        return {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    }

    double dot(const std::vector<double>& a, const std::vector<double>& b)
    {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    double norm(const std::vector<double>& v)
    {
        return std::sqrt(dot(v, v));
    }

    double angleBetween(const std::vector<double>& v1, const std::vector<double>& v2)
    {
        double n1 = norm(v1);
        double n2 = norm(v2);
        if (n1 == 0.0 || n2 == 0.0) return 0.0;

        double c = dot(v1, v2) / (n1 * n2);
        c = std::clamp(c, -1.0, 1.0);
        return std::acos(c);
    }

    double fingerCurl(
        const std::vector<double>& mcp,
        const std::vector<double>& pip,
        const std::vector<double>& dip,
        const std::vector<double>& tip)
    {
        auto v1 = vec(mcp, pip);
        auto v2 = vec(pip, dip);
        auto v3 = vec(dip, tip);

        double a1 = angleBetween(v1, v2);
        double a2 = angleBetween(v2, v3);

        return std::clamp((a1 + a2) / M_PI, 0.0, 1.0);
    }

    std::vector<double> getPoint(const std::vector<double>& d, int i)
    {
        return {d[i*3], d[i*3+1], d[i*3+2]};
    }

    void landmarkCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 63) return;

        auto thumb  = fingerCurl(getPoint(msg->data,1), getPoint(msg->data,2), getPoint(msg->data,3), getPoint(msg->data,4));
        auto index  = fingerCurl(getPoint(msg->data,5), getPoint(msg->data,6), getPoint(msg->data,7), getPoint(msg->data,8));
        auto middle = fingerCurl(getPoint(msg->data,9), getPoint(msg->data,10), getPoint(msg->data,11), getPoint(msg->data,12));
        auto ring   = fingerCurl(getPoint(msg->data,13), getPoint(msg->data,14), getPoint(msg->data,15), getPoint(msg->data,16));
        auto pinky  = fingerCurl(getPoint(msg->data,17), getPoint(msg->data,18), getPoint(msg->data,19), getPoint(msg->data,20));

        std::vector<double> fingers = {thumb, index, middle, ring, pinky};

        std::vector<double> joints;
        for (auto v : fingers)
        {
            joints.push_back(v * 1.0);
            joints.push_back(v * 1.2);
            joints.push_back(v * 1.4);
        }

        for (size_t i = 0; i < joints.size(); i++)
        {
            joints[i] = alpha_ * previous_joints_[i] + (1.0 - alpha_) * joints[i];
            previous_joints_[i] = joints[i];
        }

        sensor_msgs::msg::JointState out;
        out.name = {
            "thumb_mcp","thumb_pip","thumb_dip",
            "index_mcp","index_pip","index_dip",
            "middle_mcp","middle_pip","middle_dip",
            "ring_mcp","ring_pip","ring_dip",
            "pinky_mcp","pinky_pip","pinky_dip"
        };

        out.position = joints;

        out.header.stamp = this->now();
        out.header.frame_id = "base_link";

        joint_pub_->publish(out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}