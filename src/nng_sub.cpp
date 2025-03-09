#include <rclcpp/rclcpp.hpp>
#include <nng/nng.h>
#include <nng/protocol/pubsub0/sub.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cstring>
#include <iostream>

class NngSubscriber : public rclcpp::Node {
public:
    NngSubscriber() : Node("nng_subscriber") {
        // Create publishers
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
        point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detect_lidar_node/detect_lidar/local_ball_pos", 10);

        // Initialize subscriber sockets
        if (!initSubscriberSocket(sock_odometry, "tcp://127.0.0.1:12345", "Odometry")) {
            return;
        }
        if (!initSubscriberSocket(sock_point, "tcp://127.0.0.1:12346", "PointStamped")) {
            nng_close(sock_odometry);
            return;
        }
    }

    ~NngSubscriber() {
        nng_close(sock_odometry);
        nng_close(sock_point);
    }

    void spin() {
        char buffer[1024];
        size_t recv_size = sizeof(buffer);

        while (rclcpp::ok()) {
            int rc = nng_recv(sock_odometry, &buffer, &recv_size, NNG_FLAG_NONBLOCK);
            if (rc == 0) {
                RCLCPP_INFO(this->get_logger(), "Received from Odometry: %zu bytes", recv_size);
                publishOdometry(buffer, recv_size);
            } else if (rc != NNG_EAGAIN) {
                RCLCPP_ERROR(this->get_logger(), "nng_recv (odometry): %s", nng_strerror(rc));
            }

            rc = nng_recv(sock_point, &buffer, &recv_size, NNG_FLAG_NONBLOCK);
            if (rc == 0) {
                RCLCPP_INFO(this->get_logger(), "Received from PointStamped: %zu bytes", recv_size);
                publishPoint(buffer, recv_size);
            } else if (rc != NNG_EAGAIN) {
                RCLCPP_ERROR(this->get_logger(), "nng_recv (point): %s", nng_strerror(rc));
            }
        }
    }

private:
    nng_socket sock_odometry;
    nng_socket sock_point;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

    bool initSubscriberSocket(nng_socket& socket, const std::string& address, const std::string& name) {
        int rc = nng_sub0_open(&socket);
        if (rc != 0) {
            RCLCPP_ERROR(this->get_logger(), "nng_sub0_open (%s): %s", name.c_str(), nng_strerror(rc));
            return false;
        }

        rc = nng_dial(socket, address.c_str(), NULL, 0);
        if (rc != 0) {
            RCLCPP_ERROR(this->get_logger(), "nng_dial (%s): %s", name.c_str(), nng_strerror(rc));
            nng_close(socket);
            return false;
        }

        rc = nng_setopt(socket, NNG_OPT_SUB_SUBSCRIBE, "", 0);
        if (rc != 0) {
            RCLCPP_ERROR(this->get_logger(), "nng_setopt (%s): %s", name.c_str(), nng_strerror(rc));
            nng_close(socket);
            return false;
        }

        int recv_timeout = NNG_DURATION_INFINITE;
        rc = nng_setopt(socket, NNG_OPT_RECVTIMEO, &recv_timeout, sizeof(recv_timeout));
        if (rc != 0) {
            RCLCPP_ERROR(this->get_logger(), "nng_setopt (recvtimeo %s): %s", name.c_str(), nng_strerror(rc));
            nng_close(socket);
            return false;
        }

        // Print success message in green
        std::cout << "\033[1;32m [" << name << "] subscriber initialized successfully.\033[0m" << std::endl;
        return true;
    }

    void publishOdometry(const char* buffer, size_t size) {
        if (size != 7 * sizeof(float)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid odometry message size: %zu", size);
            return;
        }

        const float* data = reinterpret_cast<const float*>(buffer);

        nav_msgs::msg::Odometry odometry;
        odometry.pose.pose.position.x = data[0];
        odometry.pose.pose.position.y = data[1];
        odometry.pose.pose.position.z = data[2];
        odometry.pose.pose.orientation.x = data[3];
        odometry.pose.pose.orientation.y = data[4];
        odometry.pose.pose.orientation.z = data[5];
        odometry.pose.pose.orientation.w = data[6];

        // Set header
        odometry.header.stamp = this->get_clock()->now();
        odometry.header.frame_id = "world";

        RCLCPP_INFO(this->get_logger(), "Parsed Odometry: x=%.6f, y=%.6f, z=%.6f, qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f",
                    odometry.pose.pose.position.x,
                    odometry.pose.pose.position.y,
                    odometry.pose.pose.position.z,
                    odometry.pose.pose.orientation.x,
                    odometry.pose.pose.orientation.y,
                    odometry.pose.pose.orientation.z,
                    odometry.pose.pose.orientation.w);

        odometry_pub_->publish(odometry);
    }

    void publishPoint(const char* buffer, size_t size) {
        if (size != 3 * sizeof(float)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid point message size: %zu", size);
            return;
        }

        const float* data = reinterpret_cast<const float*>(buffer);

        geometry_msgs::msg::PointStamped point;
        point.point.x = data[0];
        point.point.y = data[1];
        point.point.z = data[2];

        // Set header
        point.header.stamp = this->get_clock()->now();
        point.header.frame_id = "world";

        RCLCPP_INFO(this->get_logger(), "Parsed Point: x=%.6f, y=%.6f, z=%.6f",
                    point.point.x,
                    point.point.y,
                    point.point.z);

        point_pub_->publish(point);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NngSubscriber>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}