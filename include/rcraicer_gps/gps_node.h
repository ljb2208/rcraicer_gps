#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "rcraicer_msgs/msg/gps_status.hpp"
#include "rcraicer_msgs/msg/gpsrf_status.hpp"
#include <string>
#include <vector>

#include "ubx_protocol.h"

class GPSNode : public rclcpp::Node
{
    public:
        GPSNode();
        ~GPSNode();

    private:

        UbxProtocol* ubxProto;

        void message_callback();        
        void configuration_callback();        

        rclcpp::Parameter portPath;
        rclcpp::Parameter baudRate;
        rclcpp::Parameter isBase;
        rclcpp::Parameter svInAcc;
        rclcpp::Parameter svInDur;
        rclcpp::Parameter frameId;
        rclcpp::Parameter msgDebug;
        rclcpp::Parameter outputRf;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::GPSSurvey>::SharedPtr gpsSurveyPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::GPSStatus>::SharedPtr gpsStatusPublisher;                
        rclcpp::Publisher<rcraicer_msgs::msg::GPSRFStatus>::SharedPtr gpsRFStatusPublisher;     
};