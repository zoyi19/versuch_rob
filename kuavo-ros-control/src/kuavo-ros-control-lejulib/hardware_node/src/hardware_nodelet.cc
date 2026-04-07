#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <signal.h>
#include "hardware_node.h"

class HardwareNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        NODELET_INFO("Initializing hardware nodelet...");
        nh = getNodeHandle();
        double controlFrequency = 1000;
        if (nh.hasParam("/sensor_frequency"))
        {
            nh.getParam("/sensor_frequency", controlFrequency);
        }
        NODELET_INFO("Sensor data frequency: %f", controlFrequency);
        hardware_node_ptr = std::make_unique<HighlyDynamic::HardwareNode>(nh, 1 / controlFrequency);
        hardware_node_ptr->init();
        control_thread = std::thread(&HardwareNodelet::controlLoop, this);

        // Register signal handlers
        signal(SIGINT, HardwareNodelet::signalHandler);
        signal(SIGTERM, HardwareNodelet::signalHandler);
        signal(SIGHUP, HardwareNodelet::signalHandler);
        
        // Store instance pointer for signal handler
        instance = this;

        NODELET_INFO("Hardware nodelet initialized.");
    }
    ~HardwareNodelet() override {
        NODELET_INFO("HardwareNodelet is shutting down...");
        hardware_node_ptr.reset();
    }


private:
    std::thread control_thread;
    void controlLoop()
    {
        hardware_node_ptr->real_init_wait();
    }
    ros::NodeHandle nh;
    std::unique_ptr<HighlyDynamic::HardwareNode> hardware_node_ptr;
    
    // Static pointer to current instance for signal handler
    static HardwareNodelet* instance;
    
    // Signal handler function
    static void signalHandler(int signum)
    {
        if (instance) {
            ROS_INFO("Received signal %d, cleaning up hardware resources...", signum);
            if (instance->hardware_node_ptr)
            {
                instance->hardware_node_ptr.reset();
            }
            ROS_INFO("Hardware resources released.");
        }
        
        // Re-raise the signal after cleanup
        signal(signum, SIG_DFL);
        raise(signum);
    }
};

// Initialize static member
HardwareNodelet* HardwareNodelet::instance = nullptr;

PLUGINLIB_EXPORT_CLASS(HardwareNodelet, nodelet::Nodelet)