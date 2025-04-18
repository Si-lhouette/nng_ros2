#include <rclcpp/rclcpp.hpp>
#include <nng/nng.h>
#include <nng/protocol/pubsub0/sub.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cstring>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

class NngSubscriber : public rclcpp::Node {
public:
    NngSubscriber() : Node("nng_subscriber") {
        // Create publishers
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100);
        point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detect_lidar_node/detect_lidar/ball_ekf_pos", 100);
        odometry_others_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/others_odom", 100);

        // Initialize subscriber sockets
        if (!initSubscriberSocket(sock_odometry, "tcp://127.0.0.1:12345", "Odometry")) {
            return;
        }
        if (!initSubscriberSocket(sock_point, "tcp://127.0.0.1:12346", "PointStamped")) {
            nng_close(sock_odometry);
            return;
        }
        if (!initSubscriberSocket(sock_odometry_others, "tcp://127.0.0.1:12347", "Odometry_others")) {
            return;
        }

        last_log_time_ = this->now(); 
    }

    ~NngSubscriber() {
        nng_close(sock_odometry);
        nng_close(sock_point);
        nng_close(sock_odometry_others);
    }

    void spin() {
        char buffer[1024];
        size_t recv_size = sizeof(buffer);

        rclcpp::WallRate loop_rate(500);  // Set loop frequency to 100Hz 

        while (rclcpp::ok()) {
            memset(buffer, 0, sizeof(buffer));
            recv_size = 7 * sizeof(float);
            int rc = nng_recv(sock_odometry, &buffer, &recv_size, NNG_FLAG_NONBLOCK);
            if (rc == 0) {
                // RCLCPP_INFO(this->get_logger(), "Received from Odometry: %zu bytes", recv_size);
                odom_count_++; 
                publishOdometry(buffer, recv_size);
            } else if (rc != NNG_EAGAIN) {
                RCLCPP_ERROR(this->get_logger(), "nng_recv (odometry): %s", nng_strerror(rc));
            }

            recv_size = 3 * sizeof(float);
            rc = nng_recv(sock_point, &buffer, &recv_size, NNG_FLAG_NONBLOCK);
            if (rc == 0) {
                // RCLCPP_INFO(this->get_logger(), "Received from PointStamped: %zu bytes", recv_size);
                point_count_++;
                publishPoint(buffer, recv_size);
            } else if (rc != NNG_EAGAIN) {
                RCLCPP_ERROR(this->get_logger(), "nng_recv (point): %s", nng_strerror(rc));
            }

            recv_size = 8 * sizeof(float);
            rc = nng_recv(sock_odometry_others, &buffer, &recv_size, NNG_FLAG_NONBLOCK);
            if (rc == 0) {
                // RCLCPP_INFO(this->get_logger(), "Received from Odometry: %zu bytes", recv_size);
                odom_others_count_++; 
                publishOdometryOthers(buffer, recv_size);
            } else if (rc != NNG_EAGAIN) {
                RCLCPP_ERROR(this->get_logger(), "nng_recv (odometry_others): %s", nng_strerror(rc));
            }

            rclcpp::Time now = this->now();
            if ((now - last_log_time_).seconds() >= 1.0) {
                RCLCPP_INFO(this->get_logger(), "Odometry Hz: %.2f, PointStamped Hz: %.2f, OdometryOthers Hz: %.2f",
                            static_cast<double>(odom_count_) / (now - last_log_time_).seconds(),
                            static_cast<double>(point_count_) / (now - last_log_time_).seconds(),
                            static_cast<double>(odom_others_count_) / (now - last_log_time_).seconds());

                odom_count_ = 0;
                point_count_ = 0;
                odom_others_count_ = 0;
                last_log_time_ = now;
            }

            // Sleep to limit CPU usage and ensure a frequency of 10Hz
            loop_rate.sleep();
        }
    }

private:
    nng_socket sock_odometry;
    nng_socket sock_point;
    nng_socket sock_odometry_others;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_others_pub_;

    rclcpp::Time last_log_time_;
    int odom_count_;
    int point_count_;
    int odom_others_count_;

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

        // for (size_t i = 0; i < 7; i++)
        // {
        //     std::cout <<"data[" << i << "]: " << data[i] << std::endl;
        // }

        // Set header
        odometry.header.stamp = this->get_clock()->now();
        odometry.header.frame_id = "world";

        // RCLCPP_INFO(this->get_logger(), "Parsed Odometry: x=%.6f, y=%.6f, z=%.6f, qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f",
        //             odometry.pose.pose.position.x,
        //             odometry.pose.pose.position.y,
        //             odometry.pose.pose.position.z,
        //             odometry.pose.pose.orientation.x,
        //             odometry.pose.pose.orientation.y,
        //             odometry.pose.pose.orientation.z,
        //             odometry.pose.pose.orientation.w);

        // std::cout << "Raw PointStamped Data:" << std::endl;
        // std::vector<uint8_t> raw_data(reinterpret_cast<const uint8_t*>(data),
        //                             reinterpret_cast<const uint8_t*>(data) + size);
        // for (size_t i = 0; i < raw_data.size(); ++i) {
        //     printf("%02X ", raw_data[i]);  // 以十六进制打印每个字节
        // }
        // printf("\n");

        odometry_pub_->publish(odometry);
    }


    void publishOdometryOthers(const char* buffer, size_t size) {
        if (size != 8 * sizeof(float)) {
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

        // for (size_t i = 0; i < 7; i++)
        // {
        //     std::cout <<"data[" << i << "]: " << data[i] << std::endl;
        // }

        // Set header
        odometry.header.stamp = this->get_clock()->now();
        odometry.header.frame_id = "world";
        odometry.child_frame_id = string("robot_") + std::to_string(int(data[7]));

        // RCLCPP_INFO(this->get_logger(), "Parsed Odometry: x=%.6f, y=%.6f, z=%.6f, qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f",
        //             odometry.pose.pose.position.x,
        //             odometry.pose.pose.position.y,
        //             odometry.pose.pose.position.z,
        //             odometry.pose.pose.orientation.x,
        //             odometry.pose.pose.orientation.y,
        //             odometry.pose.pose.orientation.z,
        //             odometry.pose.pose.orientation.w);

        // std::cout << "Raw PointStamped Data:" << std::endl;
        // std::vector<uint8_t> raw_data(reinterpret_cast<const uint8_t*>(data),
        //                             reinterpret_cast<const uint8_t*>(data) + size);
        // for (size_t i = 0; i < raw_data.size(); ++i) {
        //     printf("%02X ", raw_data[i]);  // 以十六进制打印每个字节
        // }
        // printf("\n");

        odometry_others_pub_->publish(odometry);
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

        // RCLCPP_INFO(this->get_logger(), "Parsed Point: x=%.6f, y=%.6f, z=%.6f",
        //             point.point.x,
        //             point.point.y,
        //             point.point.z);

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
