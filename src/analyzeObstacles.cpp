#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

class PolarScanInfo {
    public:
        float d;
        float theta;

        // Default constructor
        PolarScanInfo() : d(0.0f), theta(0.0f) {}

        // Parameterized constructor
        PolarScanInfo(const float& d, const float& theta) {
            this->d = d;
            this->theta = theta;
        }
};

// Helpers
float radToDeg(float radians) {
    return radians * (180.0f / M_PI);
}

float DegToRad(float degrees) {
    return degrees * (M_PI/ 180.0f);
}

class obstacleRangeOfDetection {
    public:
        float left;
        float right;
        float leftMid;
        float rightMid;

    obstacleRangeOfDetection(const float& rangeAngleInDegrees) {
        this->left = DegToRad(rangeAngleInDegrees);
        this->leftMid = 0;
        this->rightMid = DegToRad(360);
        this->right = DegToRad(360 - rangeAngleInDegrees);
    }
};

class ScanAnalyser: public rclcpp::Node {
    public:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
        nav_msgs::msg::Odometry current_pose;
        sensor_msgs::msg::LaserScan scan_data;
        PolarScanInfo minObstacleRange;
        float rmin = 0.0f;
        float phimin = 0.0f;
        float rstop = 0.20f;
        float rsafe = 0.30f;
        float rturn = 0.25f;
        float scaled_r = 0.0f;
        float vmax = 0.07f;
        float vreverse = -0.07f;
        float angular_speed_max = 0.2; // rad/s, adjust based on tuning
        obstacleRangeOfDetection* ObstacleRange = new obstacleRangeOfDetection(45.0);

        enum STATES {
            UNKNOWN = 0,
            FULL_SPEED = 1,
            SLOW_DOWN = 2,
            STOP = 3
        };

        enum TURNING_STATES {
            NOT_KNOWN = 0,
            GO_STRAIGHT = 1,
            TURN_LEFT = 2,
            TURN_RIGHT = 3
        };

        STATES robotState = UNKNOWN;
        TURNING_STATES robotTurnState = NOT_KNOWN;

        // Constructor
        ScanAnalyser(): Node("scan_analyzer") {
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, bind(&ScanAnalyser::scan_callback, this, placeholders::_1));
            timer_ = this->create_wall_timer(500ms, std::bind(&ScanAnalyser::initiateObstacleAvoidance, this));
        }

        // This callback method reads the scans and just defines the current STATES respective to the obstacle
        void scan_callback(const sensor_msgs::msg::LaserScan& scan_info) {
            this->scan_data = scan_info;
            float beta = 0.55;

            size_t range_size = this->scan_data.ranges.size();
            float min_scaled_distance = std::numeric_limits<float>::infinity();
            size_t min_index = 0;

            for (size_t i = 0; i < range_size; ++i) {
                float r = this->scan_data.ranges[i];

                // Validate reading
                if (!std::isfinite(r) || r < this->scan_data.range_min || r > this->scan_data.range_max) {
                    continue; // skip invalid or too close readings
                }
                
                if (r < min_scaled_distance) {
                    min_scaled_distance = r;
                    min_index = i;
                }
            }

            if (std::isfinite(min_scaled_distance)) {
                // Set properties rmin and phimin as per assignment
                float rmin = this->scan_data.ranges[min_index];
                float phimin = this->scan_data.angle_min + min_index * this->scan_data.angle_increment;

                // Store or use these properties as needed, for example:
                this->rmin = rmin;
                this->phimin = phimin;
                this->scaled_r = this->rmin * (1.0f - beta * std::cos(this->phimin));
                cout << "Cosine value << " << this->phimin << endl;

                RCLCPP_INFO(this->get_logger(),
                            "Minimum scaled distance obstacle: rmin=%.3f m, phimin=%.3f degree",
                            this->scaled_r, radToDeg(phimin));

                if(this->scaled_r > rsafe) {
                    robotState = FULL_SPEED;
                    robotTurnState = GO_STRAIGHT;
                }
                else if(this->scaled_r > rstop && this->scaled_r <= rsafe) {
                    robotState = SLOW_DOWN;

                    if(this->phimin >= this->ObstacleRange->leftMid && this->phimin <= this->ObstacleRange->left) {
                        cout << "Obstacle on Left" << endl;
                        robotTurnState = TURN_RIGHT;
                    } else if(this->phimin >= this->ObstacleRange->right && this->phimin <= this->ObstacleRange->rightMid) {
                        cout << "Obstacle on right" << endl;
                        robotTurnState = TURN_LEFT;
                    }
                }
                else if(this->scaled_r <= rstop) {
                    robotState = STOP;
                    cout << "Obstacle below the safe range, I'm Stopping. Tell me What to do next! " << endl;
                    if(this->phimin >= this->ObstacleRange->leftMid && this->phimin <= this->ObstacleRange->left) {
                        cout << "Obstacle on Left" << endl;
                        robotTurnState = TURN_RIGHT;
                    } else if(this->phimin >= this->ObstacleRange->right && this->phimin <= this->ObstacleRange->rightMid) {
                        cout << "Obstacle on right" << endl;
                        robotTurnState = TURN_LEFT;
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "No valid obstacle detected.");
            }
        }

        // Based on the STATES, the robot acts. 
        void initiateObstacleAvoidance() {
            switch(this->robotState) {
                case UNKNOWN:
                    this->handleRobotVelocity(0.0f);
                    break;

                case FULL_SPEED:
                    this->handleRobotVelocity(this->vmax);
                    break;

                case SLOW_DOWN: {
                    float slowingDownVelocity = (this->scaled_r - this->rstop)/(this->rsafe - this->rstop) * this->vmax;
                    this->handleRobotVelocity(slowingDownVelocity);
                    break;
                }

                case STOP:
                    cout << "STOP " << endl;
                    this->handleRobotVelocity(0.0f);
                    break;

                default:
                    cout << "UNKOWN STATE" << endl;
                    this->handleRobotVelocity(0.0f);
                    break;
            }
        }

        // Handles velocities
        void handleRobotVelocity(const float& linear_velocity) {

            cout << linear_velocity << endl;
            geometry_msgs::msg::Twist twist_message;
            twist_message.linear.x = linear_velocity;
            twist_message.linear.y = 0;
            twist_message.linear.z = 0;
            twist_message.angular.x = 0;
            twist_message.angular.y = 0;

            
            switch (robotTurnState) {
                case TURN_LEFT:
                    twist_message.angular.z = angular_speed_max * ((this->rsafe - this->scaled_r)/(this->rsafe - this->rturn));
                    break;
                case TURN_RIGHT:
                    twist_message.angular.z = -angular_speed_max * ((this->rsafe - this->scaled_r)/(this->rsafe - this->rturn));
                    break;
                case GO_STRAIGHT:
                default:
                    twist_message.angular.z = 0.0;
                    break;
            }

            cout << "Turning state -> " << robotTurnState << endl;
            publisher_->publish(twist_message);
        }
        
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanAnalyser>());

    rclcpp::shutdown();
    return 0;
}