#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>

class ImuEigenNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Состояние системы
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Quaterniond orientation_;
    
    // Bias акселерометра (только акселерометра, без гравитации)
    Eigen::Vector3d accel_bias_;
    
    // Для оценки bias
    std::deque<Eigen::Vector3d> accel_buffer_;
    std::deque<Eigen::Quaterniond> orientation_buffer_;
    bool bias_initialized_;
    double start_time_;
    
    double last_time_;
    bool first_message_;
    
    // Параметры
    double gravity_;
    double bias_init_duration_;
    bool use_manual_bias_;

public:
    ImuEigenNode() : Node("imu_eigen_node") {
        // Параметры
        this->declare_parameter("gravity", 9.81);
        this->declare_parameter("bias_init_duration", 2.0);
        this->declare_parameter("use_manual_bias", false);
        
        // Ручные значения bias
        this->declare_parameter("accel_bias_x", 0.0);
        this->declare_parameter("accel_bias_y", 0.0);
        this->declare_parameter("accel_bias_z", 0.0);
        
        gravity_ = this->get_parameter("gravity").as_double();
        bias_init_duration_ = this->get_parameter("bias_init_duration").as_double();
        use_manual_bias_ = this->get_parameter("use_manual_bias").as_bool();
        
        if (use_manual_bias_) {
            accel_bias_ = Eigen::Vector3d(
                this->get_parameter("accel_bias_x").as_double(),
                this->get_parameter("accel_bias_y").as_double(),
                this->get_parameter("accel_bias_z").as_double()
            );
            bias_initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Using MANUAL accel bias: [%.4f, %.4f, %.4f]", 
                accel_bias_.x(), accel_bias_.y(), accel_bias_.z());
        } else {
            accel_bias_.setZero();
            bias_initialized_ = false;
            RCLCPP_INFO(this->get_logger(), "Using AUTO accel bias calibration (%.1f seconds)", bias_init_duration_);
        }

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 100,
            std::bind(&ImuEigenNode::imuCallback, this, std::placeholders::_1)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu/eigen_pose", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        position_.setZero();
        velocity_.setZero();
        orientation_.setIdentity();
        
        last_time_ = 0.0;
        first_message_ = true;
        start_time_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Eigen IMU Node started. Using BUILT-IN orientation from IMU.");
        RCLCPP_INFO(this->get_logger(), "Waiting for /imu/data...");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        if (first_message_) {
            last_time_ = current_time;
            start_time_ = current_time;
            first_message_ = false;
            if (!use_manual_bias_) {
                RCLCPP_INFO(this->get_logger(), "First IMU message received. Calibrating accel bias...");
            }
            return;
        }

        double dt = current_time - last_time_;
        if (dt <= 0.0 || dt > 0.1) {
            last_time_ = current_time;
            return;
        }

        // === Читаем ориентацию из IMU (встроенный DSP) ===
        Eigen::Quaterniond orientation_from_imu(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z
        );
        
        // Проверка валидности кватерниона
        double q_norm = orientation_from_imu.norm();
        if (q_norm < 0.1) {
            RCLCPP_WARN(this->get_logger(), "Invalid orientation from IMU (norm=%.3f), skipping", q_norm);
            last_time_ = current_time;
            return;
        }
        orientation_from_imu.normalize();

        // Читаем ускорение
        Eigen::Vector3d accel(msg->linear_acceleration.x, 
                              msg->linear_acceleration.y, 
                              msg->linear_acceleration.z);

        // === АВТОКАЛИБРОВКА bias акселерометра ===
        if (!bias_initialized_) {
            accel_buffer_.push_back(accel);
            orientation_buffer_.push_back(orientation_from_imu);
            
            double elapsed = current_time - start_time_;
            if (elapsed >= bias_init_duration_) {
                // В статике: accel = R^T * [0, 0, -g] + bias
                // bias = accel_avg - R^T * [0, 0, -g]
                Eigen::Vector3d accel_avg = Eigen::Vector3d::Zero();
                for (const auto& a : accel_buffer_) accel_avg += a;
                accel_avg /= accel_buffer_.size();
                
                // Усредняем ориентацию (простое среднее кватернионов с нормализацией)
                Eigen::Quaterniond avg_q(0, 0, 0, 0);
                for (const auto& q : orientation_buffer_) {
                    // Учитываем знак кватерниона (q и -q одинаковы)
                    if (avg_q.coeffs().dot(q.coeffs()) < 0) {
                        avg_q.coeffs() -= q.coeffs();
                    } else {
                        avg_q.coeffs() += q.coeffs();
                    }
                }
                avg_q.normalize();
                
                // Вычисляем bias
                Eigen::Vector3d gravity_world(0.0, 0.0, -gravity_);
                Eigen::Vector3d gravity_in_body = avg_q.inverse() * gravity_world;
                accel_bias_ = accel_avg - gravity_in_body;
                
                RCLCPP_INFO(this->get_logger(), 
                    "Calibrated accel bias: [%.4f, %.4f, %.4f] m/s²",
                    accel_bias_.x(), accel_bias_.y(), accel_bias_.z());
                RCLCPP_INFO(this->get_logger(), 
                    "Initial orientation: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                    avg_q.w(), avg_q.x(), avg_q.y(), avg_q.z());
                
                bias_initialized_ = true;
                accel_buffer_.clear();
                orientation_buffer_.clear();
                
                RCLCPP_INFO(this->get_logger(), "Calibration complete! Starting integration...");
            }
            
            last_time_ = current_time;
            return;
        }

        // === ИНТЕГРАЦИЯ ===
        // 1. Используем ориентацию от IMU напрямую (без интеграции гироскопа!)
        orientation_ = orientation_from_imu;

        // 2. Вычитаем bias акселерометра
        accel -= accel_bias_;

        // 3. Компенсация гравитации
        // Гравитация в мировой системе: [0, 0, -g]
        // Переводим в систему тела и вычитаем
        Eigen::Vector3d gravity_world(0.0, 0.0, -gravity_);
        Eigen::Vector3d gravity_body = orientation_.inverse() * gravity_world;
        accel -= gravity_body;

        // 4. Переводим ускорение в мировую систему
        Eigen::Vector3d accel_world = orientation_ * accel;
        
        // 5. Интегрирование
        velocity_ += accel_world * dt;
        position_ += velocity_ * dt;

        last_time_ = current_time;

        publishTF(msg->header.stamp);
        publishPose(msg->header.stamp);
    }

    void publishTF(const builtin_interfaces::msg::Time &stamp) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "imu_base";

        t.transform.translation.x = position_.x();
        t.transform.translation.y = position_.y();
        t.transform.translation.z = position_.z();

        t.transform.rotation.x = orientation_.x();
        t.transform.rotation.y = orientation_.y();
        t.transform.rotation.z = orientation_.z();
        t.transform.rotation.w = orientation_.w();

        tf_broadcaster_->sendTransform(t);
    }

    void publishPose(const builtin_interfaces::msg::Time &stamp) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "odom";
        
        pose_msg.pose.position.x = position_.x();
        pose_msg.pose.position.y = position_.y();
        pose_msg.pose.position.z = position_.z();
        
        pose_msg.pose.orientation.x = orientation_.x();
        pose_msg.pose.orientation.y = orientation_.y();
        pose_msg.pose.orientation.z = orientation_.z();
        pose_msg.pose.orientation.w = orientation_.w();
        
        pose_pub_->publish(pose_msg);

        static double last_log = 0;
        double now = stamp.sec + stamp.nanosec * 1e-9;
        if (now - last_log > 2.0) {
            RCLCPP_INFO(this->get_logger(), 
                "Pos: [%.3f, %.3f, %.3f] | Vel: [%.3f, %.3f, %.3f]",
                position_.x(), position_.y(), position_.z(),
                velocity_.x(), velocity_.y(), velocity_.z()
            );
            last_log = now;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuEigenNode>());
    rclcpp::shutdown();
    return 0;
}