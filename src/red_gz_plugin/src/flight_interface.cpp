#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <actuator_msgs/msg/actuators.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>

#include <gz/common/Console.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

using namespace std::string_literals;

namespace red {

class FlightInterface : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate,
                        public gz::sim::ISystemPostUpdate {
  public:
    FlightInterface() = default;
    ~FlightInterface() override {
        if (this->executor_) {
            this->executor_->cancel();
        }
        if (this->spin_thread_.joinable()) {
            this->spin_thread_.join();
        }
        this->executor_.reset();

        this->ros_node_.reset();

        /**
         * Do NOT call rclcpp::shutdown() here.
         * This system plugin runs inside Gazebo's process alongside other plugins that may also
         * create ROS 2 nodes on the shared default rclcpp context. Shutting down the global
         * context would terminate those nodes as well. We instead destroy only our own ROS
         * objects above (publishers, subscribers, node) so their resources are released.
         */
    }

    void Configure(const gz::sim::Entity &_entity, const sdf::ElementConstPtr &,
                   gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override {
        gz::sim::Model model(_entity);
        if (!model.Valid(_ecm)) {
            gzerr << "[FlightInterface] Invalid model entity." << std::endl;
            return;
        }

        this->model_name_ = model.Name(_ecm);
        if (this->model_name_.empty()) {
            gzerr << "[FlightInterface] Model name is empty." << std::endl;
            return;
        }

        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__ns:="s + "/"s + this->model_name_});
        options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        this->ros_node_ = std::make_shared<rclcpp::Node>("flight_interface", options);

        // Note: Motor model subscribers are on `/<model>/command/motor_speed`
        this->motor_pub_ = this->gz_node_.Advertise<gz::msgs::Actuators>(
            "/" + this->model_name_ + "/" + this->gz_cmd_motor_topic_);
        if (!this->motor_pub_) {
            gzerr << "[FlightInterface] Failed to advertise gz topic motor command" << std::endl;
        }

        // Motor commands are high-rate "latest value wins" signals.
        // KeepLast(1) drops stale commands and minimizes latency,
        // best_effort avoids blocking and is compatible with best-effort publishers.
        this->motor_sub_ = this->ros_node_->create_subscription<actuator_msgs::msg::Actuators>(
            this->ros_cmd_motor_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&FlightInterface::MotorCommandCallback, this, std::placeholders::_1));

        // Spin ROS node in a separate thread
        this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executor_->add_node(this->ros_node_);
        this->spin_thread_ = std::thread([this]() { this->executor_->spin(); });
    }

    /**
     * PreUpdate: called every simulation iteration before physics integration.
     * Gazebo Sim (gz-physics) performs physics integration between PreUpdate and PostUpdate.
     */
    void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &) override {
        (void)info; // Do not remove this line.
    }

    /**
     * PostUpdate: called every simulation iteration after physics integration.
     */
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &) override {
        if (!this->motor_pub_ || !this->ros_node_) {
            return;
        }

        std::array<double, 4> speeds;
        {
            std::lock_guard<std::mutex> lock(this->motor_mutex_);
            speeds = this->motor_speeds_;
        }

        gz::msgs::Actuators msg;
        for (std::size_t i = 0; i < this->motor_count_; ++i) {
            msg.add_velocity(speeds[i]);
        }

        const auto sim_time_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
        auto *stamp = msg.mutable_header()->mutable_stamp();
        stamp->set_sec(static_cast<int32_t>(sim_time_ns / 1000000000));
        stamp->set_nsec(static_cast<int32_t>(sim_time_ns % 1000000000));

        this->motor_pub_.Publish(msg);
    }

  private:
    void MotorCommandCallback(const actuator_msgs::msg::Actuators::SharedPtr msg) {
        if (!msg) {
            return;
        }
        std::lock_guard<std::mutex> lock(this->motor_mutex_);
        this->motor_speeds_ = {msg->velocity[0], msg->velocity[1], msg->velocity[2],
                               msg->velocity[3]};
    }

    std::string model_name_;

    // Gazebo node
    gz::transport::Node gz_node_;

    // ROS node
    rclcpp::Node::SharedPtr ros_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;

    // Motor topic
    std::string gz_cmd_motor_topic_ = "command/motor_speed";
    std::string ros_cmd_motor_topic_ = "command/motor_speed";
    gz::transport::Node::Publisher motor_pub_;
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr motor_sub_;

    // Motor speeds
    std::mutex motor_mutex_;
    std::array<double, 4> motor_speeds_ = {0.0, 0.0, 0.0, 0.0};
    const std::size_t motor_count_ = 4;
};

} // namespace red

GZ_ADD_PLUGIN(red::FlightInterface, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(red::FlightInterface, "red::FlightInterface")
