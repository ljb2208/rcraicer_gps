#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "rcraicer_msgs/msg/gps_status.hpp"
#include <string>
#include <vector>
#include <mutex>

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
        rclcpp::Parameter isBase;
        rclcpp::Parameter frameId;
        rclcpp::Parameter msgDebug;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::GPSSurvey>::SharedPtr gpsSurveyPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::GPSStatus>::SharedPtr gpsStatusPublisher;

        void serial_data_callback(const uint8_t data);
        std::mutex dataMutex; ///< mutex for accessing incoming data
};