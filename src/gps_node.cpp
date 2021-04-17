#include "../include/rcraicer_gps/gps_node.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

GPSNode::GPSNode() : Node("gps_node"), serialPort(NULL), ubxProto(NULL)
{    
    this->get_parameter_or("serial_port", portPath, rclcpp::Parameter("serial_port", "/dev/ttyACM0"));    
    this->get_parameter_or("baud_rate", baudRate, rclcpp::Parameter("baud_rate", 460800));

    ubxProto = new UbxProtocol();

    serialPort = new SerialPort(portPath.as_string(), baudRate.as_int());
    serialPort->registerDataCallback(std::bind(&GPSNode::serial_data_callback, this, std::placeholders::_1));

    if (serialPort->isConnected())
    {
        RCLCPP_INFO(this->get_logger(), "Connected on %s @ %i", portPath.as_string().c_str(), 
                                    baudRate.as_int());
    }

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


}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNode>());
    rclcpp::shutdown();
    return 0;
}