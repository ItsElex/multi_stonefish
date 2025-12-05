#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("depth_odom_publisher")
    {
        // Crear un publicador en el tópico /odom
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry_depth", 10);

        // Crear un temporizador que llame al callback para publicar cada 100 ms (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomPublisher::publish_odometry, this));
    }

private:
    void publish_odometry()
    {
        auto message = nav_msgs::msg::Odometry();
        
        // Asignar datos ficticios para la odometría
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "odom";
        message.child_frame_id = "base_link";

        // Asignar posiciones
        message.pose.pose.position.x = 0.0;
        message.pose.pose.position.y = 0.0;
        message.pose.pose.position.z = -1.0;

        // Asignar orientación (cuaternión)
        message.pose.pose.orientation.x = 0.0;
        message.pose.pose.orientation.y = 0.0;
        message.pose.pose.orientation.z = 0.0;
        message.pose.pose.orientation.w = 1.0;

        // Asignar velocidades
        message.twist.twist.linear.x = 0.1;
        message.twist.twist.linear.y = 0.0;
        message.twist.twist.linear.z = 0.0;
        message.twist.twist.angular.x = 0.0;
        message.twist.twist.angular.y = 0.0;
        message.twist.twist.angular.z = 0.1;

        // Publicar el mensaje
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Depth published: %f", message.pose.pose.position.z);

    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

