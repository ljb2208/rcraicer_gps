#include "../include/rcraicer_gps/gps_node.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

GPSNode::GPSNode() : Node("gps_node"), ubxProto(NULL)
{    
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 460800);    
    this->declare_parameter<bool>("base", true);
    this->declare_parameter<bool>("msg_debug", false);
    this->declare_parameter<int>("base_svin_acc_limit", 2000); // millimeters
    this->declare_parameter<int>("base_svin_min_duration", 120); // seconds

    this->declare_parameter<std::string>("frame_id", "gps_link");

    // init parameters
    portPath = this->get_parameter("serial_port");
    baudRate = this->get_parameter("baud_rate");
    isBase = this->get_parameter("base");
    svInAcc = this->get_parameter("base_svin_acc_limit");
    svInDur = this->get_parameter("base_svin_min_duration");
    frameId = this->get_parameter("frame_id");
    msgDebug = this->get_parameter("msg_debug");    


    std::string topicPrefix = "rover_";

    if (isBase.as_bool() == true)
        topicPrefix = "base_";

    navSatFixPublisher = this->create_publisher<sensor_msgs::msg::NavSatFix>(topicPrefix + "navsat_fix", 10);
    gpsStatusPublisher = this->create_publisher<rcraicer_msgs::msg::GPSStatus>(topicPrefix + "gps_status", 10);    

    if (isBase.as_bool() == true)
        gpsSurveyPublisher = this->create_publisher<rcraicer_msgs::msg::GPSSurvey>(topicPrefix + "gps_survey", 10);
    

    ubxProto = new UbxProtocol(msgDebug.as_bool(), portPath.as_string(), baudRate.as_int(), isBase.as_bool());
    ubxProto->registerMessageCallback(std::bind(&GPSNode::message_callback, this));
    ubxProto->registerConfigurationCallback(std::bind(&GPSNode::configuration_callback, this));

    if (ubxProto->connect())
    {
        RCLCPP_INFO(this->get_logger(), "Serial Port connected on %s at %i", portPath.as_string().c_str(), baudRate.as_int());
    }

    ubxProto->configure();

    if (isBase.as_bool() == true)
    {
        ubxProto->configureSurveyIn((uint32_t)svInDur.as_int(), (uint32_t)svInAcc.as_int());
    }

}

GPSNode::~GPSNode()
{    
    if (ubxProto != NULL)
        delete ubxProto;
}

void GPSNode::configuration_callback()
{
    if (ubxProto->ackNackMessageReady())
    {
        if (ubxProto->getAckStatus())
        {
            RCLCPP_INFO(this->get_logger(), "Configuration successful");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Configuration failed");
        }
    }
}

void GPSNode::message_callback()
{
    // int ret = ubxProto->parseChar(data);

    if (ubxProto->gpsStatusMessageReady())
    {            
        rcraicer_msgs::msg::GPSStatus msg = ubxProto->getGpsStatusMessage();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frameId.as_string();

        gpsStatusPublisher->publish(msg);            
    }

    if (ubxProto->navSatFixMessageReady())
    {
        sensor_msgs::msg::NavSatFix msg = ubxProto->getNavSatFixMessage();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frameId.as_string();

        navSatFixPublisher->publish(msg);
    }        

    if (ubxProto->gpsSurveyMessageReady() && isBase.as_bool() == true)
    {
        rcraicer_msgs::msg::GPSSurvey msg = ubxProto->getGpsSurveyMessage();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frameId.as_string();

        gpsSurveyPublisher->publish(msg);        
    }    

    if (ubxProto->warningMessageReady())
    {
        std::string msg = ubxProto->getWarningMessage();
        RCLCPP_WARN(this->get_logger(), "Warning/Error: %s", msg.c_str());
    }

    if (ubxProto->debugMessageReady())
    {
        std::string msg = ubxProto->getDebugMessage();
        RCLCPP_INFO(this->get_logger(), "Debug/Notice: %s", msg.c_str());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNode>());
    rclcpp::shutdown();
    return 0;
}