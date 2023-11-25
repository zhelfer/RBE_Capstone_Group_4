	#include <px4_msgs/msg/offboard_control_mode.hpp>
    #include <px4_msgs/msg/trajectory_setpoint.hpp>
    #include <px4_msgs/msg/timesync.hpp>
    #include <px4_msgs/msg/vehicle_command.hpp>
    #include <px4_msgs/msg/vehicle_control_mode.hpp>
    #include <px4_msgs/msg/vehicle_local_position.hpp>
    #include <rclcpp/rclcpp.hpp>

    #include <stdint.h>
    #include <chrono>
    #include <iostream>
    #include "std_msgs/msg/string.hpp"
    #include <math.h>

    float X;
    float Y;    

    using namespace std::chrono;
    using namespace std::chrono_literals;
    using namespace px4_msgs::msg;
    class setpoint : public rclcpp::Node {
    public:
        setpoint() : Node("setpoint") {
        
            offboard_control_mode_publisher_ =
                this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
            trajectory_setpoint_publisher_ =
                this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
            vehicle_command_publisher_ =
                this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

            // get common timestamp
            timesync_sub_ =
                this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
                    [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                        timestamp_.store(msg->timestamp);
                    });

            offboard_setpoint_counter_ = 0;

        

            auto sendCommands = [this]() -> void {
                if (offboard_setpoint_counter_ == 10) {
                    // Change to Offboard mode after 10 setpoints
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                    // Arm the vehicle
                    this->arm();

                }
    //-------------
                subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/vehicle_local_position/out",
    #ifdef ROS_DEFAULT_API
                10,
    #endif
                [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
                X = msg->x;
                Y = msg->y;
                std::cout << "\n\n\n\n\n\n\n\n\n\n";
                std::cout << "RECEIVED VEHICLE GPS POSITION DATA"   << std::endl;
                std::cout << "=================================="   << std::endl;
                std::cout << "ts: "      << msg->timestamp    << std::endl;
                //std::cout << "lat: " << msg->x  << std::endl;
                //std::cout << "lon: " << msg->y << std::endl;
                std::cout << "lat: " << X  << std::endl;
                std::cout << "lon: " << Y << std::endl;
                std::cout << "waypoint: " << waypoints[waypointIndex][0] << std::endl;
                std::cout << "waypoint: " << waypoints[waypointIndex][1] << std::endl;
                if((waypoints[waypointIndex][0] + 0.3 > X && waypoints[waypointIndex][0] - 0.3 < X)&&(waypoints[waypointIndex][1] + 0.3 > Y && waypoints[waypointIndex][1] - 0.3 < Y)){
                waypointIndex++;
                if (waypointIndex >= waypoints.size())
                    exit(0);
                    //waypointIndex = 0;

                RCLCPP_INFO(this->get_logger(), "Next waypoint: %.2f %.2f %.2f", waypoints[waypointIndex][0], waypoints[waypointIndex][1], waypoints[waypointIndex][2]);
            }
            });
    //--------------

                        // offboard_control_mode needs to be paired with trajectory_setpoint
                publish_offboard_control_mode();
                publish_trajectory_setpoint();

                    // stop the counter after reaching 11
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }
            };

        /*
            auto nextWaypoint = [this]() -> void {
                
                waypointIndex++; 

                if (waypointIndex >= waypoints.size()) 
                    waypointIndex = 0;

                RCLCPP_INFO(this->get_logger(), "Next waypoint: %.2f %.2f %.2f", waypoints[waypointIndex][0], waypoints[waypointIndex][1], waypoints[waypointIndex][2]);
            };
            */
            commandTimer = this->create_wall_timer(100ms, sendCommands);
            //waypointTimer = this->create_wall_timer(2s, nextWaypoint); //EA   
        }

        void arm() const;
        void disarm() const;
        void topic_callback() const;
    private:
        

    std::vector<std::vector<float>> waypoints = {{0,0,-5,},
    {2,0,-5,},
    {2.35216,0.476806,-5,},
    {2.57897,1.09037,-5,},
    {2.64107,1.80686,-5,},
    {2.50814,2.58248,-5,},
    {2.16121,3.36588,-5,},
    {1.59437,4.10097,-5,},
    {0.815842,4.73016,-5,},
    {-0.151838,5.19778,-5,},
    {-1.27233,5.45355,-5,},
    {-2.49688,5.45578,-5,},
    {-3.76641,5.17438,-5,},
    {-5.01428,4.59315,-5,},
    {-6.1696,3.71161,-5,},
    {-7.16089,2.54591,-5,},
    {-7.91994,1.12896,-5,},
    {-8.38568,-0.490343,-5,},
    {-8.50782,-2.24876,-5,},
    {-8.25018,-4.07119,-5,},
    {-7.59329,-5.87384,-5,},
    {-6.53644,-7.56803,-5,},
    {-5.09871,-9.06439,-5,},
    {-3.31919,-10.2773,-5,},
    {-1.25611,-11.1293,-5,},
    {1.01499,-11.5555,-5,},
    {3.40395,-11.5071,-5,},
    {5.8096,-10.9548,-5,},
    {8.12407,-9.89139,-5,},
    {10.2375,-8.33272,-5,},
    {12.0431,-6.31859,-5,},
    {13.4424,-3.91182,-5,},
    {14.3502,-1.19649,-5,},
    {14.6991,1.72493,-5,},
    {14.4435,4.73543,-5,},
    {13.5626,7.70817,-5,},
    {12.0624,10.5118,-5,},
    {9.97696,13.0162,-5,},
    {7.36759,15.0983,-5,},
    {4.32167,16.6482,-5,},
    {0.949612,17.5744,-5,},
    {-2.619,17.8084,-5,},
    {-6.24045,17.3094,-5,},
    {-9.76262,16.0665,-5,},
    {-13.0314,14.1004,-5,},
    {-15.8974,11.4644,-5,},
    {-18.2226,8.24237,-5,},
    {-19.8868,4.54696,-5,},
    {-20.7936,0.515337,-5,},
    {-20.8754,-3.69574,-5,},
    {-20.0972,-7.91595,-5,},
    {-20.8754,-3.69574,-5,},
    {-20.7936,0.515337,-5,},
    {-19.8868,4.54696,-5,},
    {-18.2226,8.24237,-5,},
    {-15.8974,11.4644,-5,},
    {-13.0314,14.1004,-5,},
    {-9.76262,16.0665,-5,},
    {-6.24045,17.3094,-5,},
    {-2.619,17.8084,-5,},
    {0.949612,17.5744,-5,},
    {4.32167,16.6482,-5,},
    {7.36759,15.0983,-5,},
    {9.97696,13.0162,-5,},
    {12.0624,10.5118,-5,},
    {13.5626,7.70817,-5,},
    {14.4435,4.73543,-5,},
    {14.6991,1.72493,-5,},
    {14.3502,-1.19649,-5,},
    {13.4424,-3.91182,-5,},
    {12.0431,-6.31859,-5,},
    {10.2375,-8.33272,-5,},
    {8.12407,-9.89139,-5,},
    {5.8096,-10.9548,-5,},
    {3.40395,-11.5071,-5,},
    {1.01499,-11.5555,-5,},
    {-1.25611,-11.1293,-5,},
    {-3.31919,-10.2773,-5,},
    {-5.09871,-9.06439,-5,},
    {-6.53644,-7.56803,-5,},
    {-7.59329,-5.87384,-5,},
    {-8.25018,-4.07119,-5,},
    {-8.50782,-2.24876,-5,},
    {-8.38568,-0.490343,-5,},
    {-7.91994,1.12896,-5,},
    {-7.16089,2.54591,-5,},
    {-6.1696,3.71161,-5,},
    {-5.01428,4.59315,-5,},
    {-3.76641,5.17438,-5,},
    {-2.49688,5.45578,-5,},
    {-1.27233,5.45355,-5,},
    {-0.151838,5.19778,-5,},
    {0.815842,4.73016,-5,},
    {1.59437,4.10097,-5,},
    {2.16121,3.36588,-5,},
    {2.50814,2.58248,-5,},
    {2.64107,1.80686,-5,},
    {2.57897,1.09037,-5,},
    {2.35216,0.476806,-5,},
    {2,0,-5,},
    {0,0,-5,},
    {0,0,0,}
    };      // Land
        

        int waypointIndex = 0;

        rclcpp::TimerBase::SharedPtr commandTimer;
        rclcpp::TimerBase::SharedPtr waypointTimer;

        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
        //
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
        //
        std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

        uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

        void publish_offboard_control_mode() const;
        void publish_trajectory_setpoint() const;
        void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                        float param2 = 0.0) const;
    };

    void setpoint::arm() const {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

        RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    void setpoint::disarm() const {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

        RCLCPP_INFO(this->get_logger(), "Disarm command send");
    }

    void setpoint::publish_offboard_control_mode() const {
        OffboardControlMode msg{};
        msg.timestamp = timestamp_.load();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_publisher_->publish(msg);
    }   

    void setpoint::publish_trajectory_setpoint() const {
        TrajectorySetpoint msg{};
        msg.timestamp = timestamp_.load();
        msg.position = {waypoints[waypointIndex][0],waypoints[waypointIndex][1],waypoints[waypointIndex][2]};
        msg.yaw = std::nanf("0");       //-3.14; // [-PI:PI]
        trajectory_setpoint_publisher_->publish(msg);
    }

    void setpoint::publish_vehicle_command(uint16_t command, float param1,
                            float param2) const {
        VehicleCommand msg{};
        msg.timestamp = timestamp_.load();
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_publisher_->publish(msg);
    }

    int main(int argc, char* argv[]) {
        std::cout << "Starting setpoint node..." << std::endl;
        
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<setpoint>());

        rclcpp::shutdown();
        return 0;
    }