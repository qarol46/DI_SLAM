#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegratedImuMeasurements.h>
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include <filesystem>

class ImuPreintegrationNode : public rclcpp::Node {
private:
    // Подписки и публикаторы
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr covariance_pub_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Сервисы
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_srv_;
    
    // GTSAM объекты
    gtsam::PreintegratedImuMeasurements::shared_ptr preintegrated_;
    gtsam::imuBias::ConstantBias current_bias_;
    
    // Состояние
    double last_imu_time_;
    double start_time_;
    bool first_imu_received_;
    int measurement_count_;
    
    // Параметры
    double gravity_;
    double sigma_g_, sigma_a_, sigma_bg_, sigma_ba_;
    std::string output_directory_;
    bool export_enabled_;
    
    // Статистика
    struct Stats {
        double max_position_error;
        double max_velocity_error;
        double max_rotation_error;
        int total_measurements;
        std::chrono::time_point<std::chrono::steady_clock> start_time;
    } stats_;
    
public:
    ImuPreintegrationNode() : Node("imu_preintegration_node") {
        // Загружаем параметры
        this->declare_parameter("gravity", 9.81);
        this->declare_parameter("sigma_g", 0.001);
        this->declare_parameter("sigma_a", 0.01);
        this->declare_parameter("sigma_bg", 0.0001);
        this->declare_parameter("sigma_ba", 0.001);
        this->declare_parameter("output_directory", "/tmp/imu_test");
        this->declare_parameter("export_enabled", true);
        this->declare_parameter("publish_rate", 10.0); // Hz
        
        gravity_ = this->get_parameter("gravity").as_double();
        sigma_g_ = this->get_parameter("sigma_g").as_double();
        sigma_a_ = this->get_parameter("sigma_a").as_double();
        sigma_bg_ = this->get_parameter("sigma_bg").as_double();
        sigma_ba_ = this->get_parameter("sigma_ba").as_double();
        output_directory_ = this->get_parameter("output_directory").as_string();
        export_enabled_ = this->get_parameter("export_enabled").as_bool();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        // Создаем директорию для экспорта
        if (export_enabled_) {
            std::filesystem::create_directories(output_directory_);
        }
        
        // Инициализация GTSAM
        auto preint_params = gtsam::PreintegrationParams::MakeSharedU(gravity_);
        preint_params->accelerometerCovariance = gtsam::Matrix33::Identity() * pow(sigma_a_, 2);
        preint_params->gyroscopeCovariance = gtsam::Matrix33::Identity() * pow(sigma_g_, 2);
        preint_params->biasAccCovariance = gtsam::Matrix33::Identity() * pow(sigma_ba_, 2);
        preint_params->biasOmegaCovariance = gtsam::Matrix33::Identity() * pow(sigma_bg_, 2);
        
        current_bias_ = gtsam::imuBias::ConstantBias();
        preintegrated_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(preint_params, current_bias_);
        
        // Подписки и публикаторы
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 100,
            std::bind(&ImuPreintegrationNode::imuCallback, this, std::placeholders::_1)
        );
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/imu/relative_pose", 10
        );
        
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/imu/relative_velocity", 10
        );
        
        covariance_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/imu/covariance_visualization", 10
        );
        
        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Сервисы
        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/imu/reset_preintegration",
            std::bind(&ImuPreintegrationNode::resetCallback, this, 
                      std::placeholders::_1, std::placeholders::_2)
        );
        
        export_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/imu/export_data",
            std::bind(&ImuPreintegrationNode::exportCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );
        
        // Таймер для публикации с заданной частотой
        auto publish_period = std::chrono::duration<double>(1.0 / publish_rate);
        auto timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
            std::bind(&ImuPreintegrationNode::publishCallback, this)
        );
        
        last_imu_time_ = 0.0;
        start_time_ = 0.0;
        first_imu_received_ = false;
        measurement_count_ = 0;
        
        // Инициализация статистики
        stats_.max_position_error = 0.0;
        stats_.max_velocity_error = 0.0;
        stats_.max_rotation_error = 0.0;
        stats_.total_measurements = 0;
        stats_.start_time = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "IMU Preintegration Node initialized");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  Gravity: %.3f", gravity_);
        RCLCPP_INFO(this->get_logger(), "  Sigma G: %.6f", sigma_g_);
        RCLCPP_INFO(this->get_logger(), "  Sigma A: %.6f", sigma_a_);
        RCLCPP_INFO(this->get_logger(), "  Sigma BG: %.6f", sigma_bg_);
        RCLCPP_INFO(this->get_logger(), "  Sigma BA: %.6f", sigma_ba_);
        RCLCPP_INFO(this->get_logger(), "  Output directory: %s", output_directory_.c_str());
    }
    
    ~ImuPreintegrationNode() {
        if (export_enabled_) {
            exportData();
        }
    }
    
private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        if (!first_imu_received_) {
            last_imu_time_ = current_time;
            start_time_ = current_time;
            first_imu_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First IMU message received at t=%.3f", current_time);
            return;
        }
        
        double dt = current_time - last_imu_time_;
        
        // Проверка на разумность dt
        if (dt <= 0.0 || dt > 0.1) {
            RCLCPP_WARN(this->get_logger(), "Invalid dt: %.6f, skipping measurement", dt);
            last_imu_time_ = current_time;
            return;
        }
        
        // Извлекаем данные IMU
        Eigen::Vector3d omega(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
        
        Eigen::Vector3d accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
        
        // Интегрируем измерение
        try {
            preintegrated_->integrateMeasurement(omega, accel, dt);
            measurement_count_++;
            stats_.total_measurements++;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error integrating IMU measurement: %s", e.what());
        }
        
        last_imu_time_ = current_time;
    }
    
    void publishCallback() {
        if (!first_imu_received_) return;
        
        // Получаем прединтегрированные значения
        gtsam::Rot3 delta_R = preintegrated_->deltaRij();
        gtsam::Vector3 delta_v = preintegrated_->deltaVij();
        gtsam::Vector3 delta_p = preintegrated_->deltaPij();
        
        // Получаем ковариацию
        gtsam::Matrix9 cov = preintegrated_->preintMeasCov();
        
        // Публикуем позицию
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "imu";
        
        pose_msg.pose.position.x = delta_p.x();
        pose_msg.pose.position.y = delta_p.y();
        pose_msg.pose.position.z = delta_p.z();
        
        gtsam::Quaternion quat = delta_R.toQuaternion();
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();
        
        pose_pub_->publish(pose_msg);
        
        // Публикуем скорость
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "imu";
        
        twist_msg.twist.linear.x = delta_v.x();
        twist_msg.twist.linear.y = delta_v.y();
        twist_msg.twist.linear.z = delta_v.z();
        
        velocity_pub_->publish(twist_msg);
        
        // Публикуем TF
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "imu";
        transform_stamped.child_frame_id = "imu_preintegrated";
        
        transform_stamped.transform.translation.x = delta_p.x();
        transform_stamped.transform.translation.y = delta_p.y();
        transform_stamped.transform.translation.z = delta_p.z();
        
        transform_stamped.transform.rotation.x = quat.x();
        transform_stamped.transform.rotation.y = quat.y();
        transform_stamped.transform.rotation.z = quat.z();
        transform_stamped.transform.rotation.w = quat.w();
        
        tf_broadcaster_->sendTransform(transform_stamped);
        
        // Логируем статистику каждые 10 секунд
        static double last_log_time = 0.0;
        double current_time = this->now().seconds();
        if (current_time - last_log_time > 10.0) {
            double elapsed = current_time - start_time_;
            RCLCPP_INFO(this->get_logger(), 
                "=== IMU Preintegration Stats (t=%.1fs) ===\n"
                "Measurements: %d\n"
                "Position: [%.3f, %.3f, %.3f] m\n"
                "Velocity: [%.3f, %.3f, %.3f] m/s\n"
                "Rotation: [%.3f, %.3f, %.3f] rad\n"
                "Position uncertainty: %.3f m\n"
                "Velocity uncertainty: %.3f m/s",
                elapsed,
                measurement_count_,
                delta_p.x(), delta_p.y(), delta_p.z(),
                delta_v.x(), delta_v.y(), delta_v.z(),
                delta_R.xyz().x(), delta_R.xyz().y(), delta_R.xyz().z(),
                sqrt(cov(6,6) + cov(7,7) + cov(8,8)),
                sqrt(cov(3,3) + cov(4,4) + cov(5,5))
            );
            last_log_time = current_time;
        }
    }
    
    void resetCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) 
    {
        // Сохраняем данные перед сбросом
        if (export_enabled_) {
            exportData();
        }
        
        preintegrated_->resetIntegration();
        measurement_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Preintegration reset");
        response->success = true;
        response->message = "Preintegration reset successfully";
    }
    
    void exportCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) 
    {
        exportData();
        response->success = true;
        response->message = "Data exported to " + output_directory_;
    }
    
    void exportData() {
        if (!export_enabled_) return;
        
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        
        std::string filename = output_directory_ + "/imu_preintegration_" + ss.str() + ".csv";
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }
        
        // Заголовки
        file << "timestamp,position_x,position_y,position_z,"
             << "velocity_x,velocity_y,velocity_z,"
             << "rotation_x,rotation_y,rotation_z,rotation_w,"
             << "covariance_position,covariance_velocity,covariance_rotation\n";
        
        // Данные
        gtsam::Rot3 delta_R = preintegrated_->deltaRij();
        gtsam::Vector3 delta_v = preintegrated_->deltaVij();
        gtsam::Vector3 delta_p = preintegrated_->deltaPij();
        gtsam::Matrix9 cov = preintegrated_->preintMeasCov();
        gtsam::Quaternion quat = delta_R.toQuaternion();
        
        file << this->now().seconds() << ","
             << delta_p.x() << "," << delta_p.y() << "," << delta_p.z() << ","
             << delta_v.x() << "," << delta_v.y() << "," << delta_v.z() << ","
             << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << ","
             << sqrt(cov(6,6) + cov(7,7) + cov(8,8)) << ","
             << sqrt(cov(3,3) + cov(4,4) + cov(5,5)) << ","
             << sqrt(cov(0,0) + cov(1,1) + cov(2,2)) << "\n";
        
        file.close();
        
        RCLCPP_INFO(this->get_logger(), "Data exported to: %s", filename.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPreintegrationNode>());
    rclcpp::shutdown();
    return 0;
}