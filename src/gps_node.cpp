#include "../include/rcraicer_gps/gps_node.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

GPSNode::GPSNode() : Node("gps_node"), serialPort(NULL), ubxProto(NULL)
{    
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 460800);
    this->declare_parameter<bool>("base", true);
    this->declare_parameter<bool>("msg_debug", true);
    this->declare_parameter<std::string>("frame_id", "gps_link");

    // init parameters
    portPath = this->get_parameter("serial_port");
    baudRate = this->get_parameter("baud_rate");
    isBase = this->get_parameter("base");
    frameId = this->get_parameter("frame_id");
    msgDebug = this->get_parameter("msg_debug");

    std::string topicPrefix = "rover_";

    if (isBase.as_bool() == true)
        topicPrefix = "base_";

    navSatFixPublisher = this->create_publisher<sensor_msgs::msg::NavSatFix>(topicPrefix + "navsat_fix", 10);
    gpsStatusPublisher = this->create_publisher<rcraicer_msgs::msg::GPSStatus>(topicPrefix + "gps_status", 10);    

    if (isBase.as_bool() == true)
        gpsSurveyPublisher = this->create_publisher<rcraicer_msgs::msg::GPSSurvey>(topicPrefix + "gps_survey", 10);
    

    ubxProto = new UbxProtocol(msgDebug.as_bool());

    serialPort = new SerialPort(portPath.as_string(), baudRate.as_int());
    

    if (serialPort->isConnected())
    {
        RCLCPP_INFO(this->get_logger(), "Connected on %s @ %i", portPath.as_string().c_str(), 
                                    baudRate.as_int());
    }

    serialPort->registerDataCallback(std::bind(&GPSNode::serial_data_callback, this, std::placeholders::_1));

}

GPSNode::~GPSNode()
{
    if (serialPort != NULL)
        delete serialPort;

    if (ubxProto != NULL)
        delete ubxProto;
}

void GPSNode::serial_data_callback(const uint8_t data)
{
    int ret = ubxProto->parseChar(data);

    if (ret != 0)
    {
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
    }    
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNode>());
    rclcpp::shutdown();
    return 0;
}