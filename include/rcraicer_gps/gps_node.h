#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <vector>

#include "serial_port.h"
#include "ubx_protocol.h"

class GPSNode : public rclcpp::Node
{
    public:
        GPSNode();
        ~GPSNode();

    private:
        SerialPort* serialPort;
        UbxProtocol* ubxProto;

        rclcpp::Parameter portPath;
        rclcpp::Parameter baudRate;

        void serial_data_callback(const uint8_t data);
};