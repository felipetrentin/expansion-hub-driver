
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "config.hpp"

#include "librhsp/include/rhsp/rhsp.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rcutils/logging_macros.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// refer to https://github.com/ros2/demos/blob/humble/lifecycle/src/lifecycle_talker.cpp



class rhspNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    rhspNode(const std::string &node_name, bool intra_process_comms = false) :
        rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {

    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_configure() is called.");
        timer_ = this->create_wall_timer(500ms, std::bind(&rhspNode::sendUpdate, this));
        floatSubscriber = this->create_subscription<std_msgs::msg::Float32>("garra", 10, std::bind(&rhspNode::servoCallback, this, _1));
        cmdSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&rhspNode::twistCallback, this, _1));
        if (!serial)
        {
            serial = new RhspSerial();
            rhsp_serialInit(serial);
            switch(rhsp_serialOpen(serial, "/dev/ttyUSB0", 460800, 8, RHSP_SERIAL_PARITY_NONE, 1, RHSP_SERIAL_FLOW_CONTROL_NONE)){
                case RHSP_SERIAL_ERROR_OPENING:
                    RCLCPP_WARN(get_logger(), "Unable to connect serial to hub!!");
                    RCLCPP_WARN(get_logger(), "Serial error, unable to open.");
                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
                    break;
                RCLCPP_INFO(get_logger(), "serial connected");
            }
        }else{
            RCLCPP_INFO(get_logger(), "serial already conected, restarting connection.");
            rhsp_close(hub);
            rhsp_serialClose(serial);
            rhsp_serialInit(serial);
            switch(rhsp_serialOpen(serial, "/dev/ttyUSB0", 460800, 8, RHSP_SERIAL_PARITY_NONE, 1, RHSP_SERIAL_FLOW_CONTROL_NONE)){
                case RHSP_SERIAL_ERROR_OPENING:
                    RCLCPP_WARN(get_logger(), "Unable to connect serial to hub (after disconnecting)!!");
                    RCLCPP_WARN(get_logger(), "Serial error, unable to open.");
                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
                    break;
            }
            RCLCPP_INFO(get_logger(), "serial connected");
        }
        RCLCPP_INFO(get_logger(), "starting hub discovery");
        RhspDiscoveredAddresses addresses;
        memset(&addresses, 0, sizeof(RhspDiscoveredAddresses));
        int discoveryResult = rhsp_discoverRevHubs(serial, &addresses);
        switch (discoveryResult){
        case RHSP_RESULT_OK:
            RCLCPP_INFO(get_logger(), "allocating parent hub");
            hub = rhsp_allocRevHub(serial, addresses.parentAddress);
            RCLCPP_INFO(get_logger(), "connected successfuly to parent");
            uint8_t ack;
            RhspModuleStatus status;
            rhsp_getModuleStatus(hub, true, &status, &ack);
            RCLCPP_INFO(get_logger(), "rhsp status: %d ack:%d",status.statusWord, ack);

            if(rhsp_setModuleLedColor(hub, 0, 255, 255, &ack) == RHSP_RESULT_OK){
                RCLCPP_INFO(get_logger(), "color change success");
            }else{
                RCLCPP_WARN(get_logger(), "color change err, returned %d", ack);
            }
            if(rhsp_setResponseTimeoutMs(hub, 700)){
                RCLCPP_INFO(get_logger(), "timeout change success");
            }else{
                RCLCPP_WARN(get_logger(), "timer change err");
            }
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            break;
        case RHSP_RESULT_ATTENTION_REQUIRED:
            RCLCPP_WARN(get_logger(), "Unable to connect to hub!!");
            RCLCPP_WARN(get_logger(), "ATTENTION_REQUIRED");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            break;
        case RHSP_RESULT_DISCOVERY_MULTIPLE_PARENTS_DETECTED:
            RCLCPP_WARN(get_logger(), "Unable to connect to hub!!");
            RCLCPP_WARN(get_logger(), "DISCOVERY_MULTIPLE_PARENTS_DETECTED");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            break;
        }
        RCLCPP_WARN(get_logger(), "unknown error. (probably no hubs found)");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    //on activate part of lifecycle
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state)
    {
        // The parent class method automatically transition on managed entities
        // (currently, LifecyclePublisher).
        // pub_->on_activate() could also be called manually here.
        // Overriding this method is optional, a lot of times the default is enough.
        LifecycleNode::on_activate(state);
        uint8_t ack;
        if(rhsp_setModuleLedColor(hub, 80, 0, 255, &ack) == RHSP_RESULT_OK){
            RCLCPP_INFO(get_logger(), "color change success");
        }else{
            RCLCPP_WARN(get_logger(), "color change err, returned %d", ack);
        }
        RCLCPP_INFO(get_logger(), "set motor modes");
        ClosedLoopControlParameters param;
        param.type = PIDF_TAG;
        param.pidf.p = Kp;
        param.pidf.i = Ki;
        param.pidf.d = Kd;
        param.pidf.f = Kf;

        for(int mot =0; mot<4; mot++){
            rhsp_setMotorChannelMode(hub, mot, MOTOR_MODE_REGULATED_VELOCITY, false, &ack);
            rhsp_setMotorTargetVelocity(hub, mot, 0, &ack);
            rhsp_setClosedLoopControlCoefficients(hub, mot, MOTOR_MODE_REGULATED_VELOCITY, &param, &ack);
            rhsp_setMotorChannelEnable(hub, mot, true, &ack);
        }
        rhsp_setServoEnable(hub, 0, true, &ack);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state)
    {
        // In our shutdown phase, we release the shared pointers to the
        // timer and publisher. These entities are no longer available
        // and our node is "clean".
        LifecycleNode::on_shutdown(state);
        timer_.reset();
        rhsp_close(hub);
        rhsp_serialClose(serial);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:

    void servoCallback(const std_msgs::msg::Float32::SharedPtr msg){
        uint8_t ack;
        rhsp_setServoPulseWidth(hub, 0, (msg->data*1000), &ack);
    }

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        uint8_t ack;
        TICK_PER_RADIANS;
        float m0 = (msg->linear.x + msg->linear.y - (L_SUM)*msg->angular.z)/WHEEL_RADIUS;
        float m1 = (msg->linear.x - msg->linear.y + (L_SUM)*msg->angular.z)/WHEEL_RADIUS;
        float m2 = (msg->linear.x - msg->linear.y - (L_SUM)*msg->angular.z)/WHEEL_RADIUS;
        float m3 = (msg->linear.x + msg->linear.y + (L_SUM)*msg->angular.z)/WHEEL_RADIUS;
        rhsp_setMotorTargetVelocity(hub, 0, m0, &ack);
        rhsp_setMotorTargetVelocity(hub, 1, m1, &ack);
        rhsp_setMotorTargetVelocity(hub, 2, m2, &ack);
        rhsp_setMotorTargetVelocity(hub, 3, m3, &ack);
    }

    void sendUpdate(){
        rhsp_sendKeepAlive(hub, nullptr);
    }

    RhspRevHub *hub;
    RhspSerial *serial;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr floatSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSubscriber;
};

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<rhspNode> lc_node =
        std::make_shared<rhspNode>("expansion_hub");

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}