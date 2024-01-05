#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tuple>

using namespace std::chrono_literals;

enum class MoveType {
    FORWARD,
    TURN,
    STOP
};

class PidMazeSolver : public rclcpp::Node {
public:    
    PidMazeSolver() : Node("distance_controller") {
        odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PidMazeSolver::odomCallback, this, std::placeholders::_1));

        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer = this->create_wall_timer(
            20ms, std::bind(&PidMazeSolver::timerCallback, this));

        lastExecutionTime = std::chrono::high_resolution_clock::now();
    }
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "X: %2f, Y: %2f", msg->pose.pose.position.x, msg->pose.pose.position.y);
        currentPosition = msg->pose.pose;
    }

    void timerCallback() {

        float deltaTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - lastExecutionTime).count() / 1000000000.0;

        if (moveType == MoveType::FORWARD && moveForwardPid(goals[goalCounter], deltaTime))
        {
            errorReset();

            moveType = MoveType::STOP;
            nextMoveType = MoveType::TURN;

            goalCounter++;
        }

        if (moveType == MoveType::TURN && turnPid(goals[goalCounter], deltaTime))
        {
            errorReset();

            moveType = MoveType::STOP;
            nextMoveType = MoveType::FORWARD;
        }

        if (moveType == MoveType::STOP)
        {
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;

            velocity_publisher->publish(cmd_vel_msg);

            moveType = nextMoveType;

            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        velocity_publisher->publish(cmd_vel_msg);
        
        if (goalCounter == goals.size()) {
            this->timer->cancel();
            rclcpp::shutdown();
            return;
        }

        lastExecutionTime = std::chrono::high_resolution_clock::now();
    }

    bool moveForwardPid(std::tuple<float, float> goal, float deltaTime) {

        float errorDistanceX = std::get<0>(goal) - currentPosition.position.x;
        float errorDistanceY = std::get<1>(goal) - currentPosition.position.y;

        float newErrorDistance = sqrt(pow(errorDistanceX, 2.0) + pow(errorDistanceY, 2.0));

        // Error for the proportional term
        float errorProportional = errorDistance;

        // Error for the integral term.
        errorIntegral = errorIntegral + newErrorDistance * deltaTime;

        // Error for the derivative term.
        float errorDerivative = (newErrorDistance - errorDistance) / deltaTime;

        // RCLCPP_INFO(this->get_logger(), "X: %2f, Y: %2f, E: %2f", abs(errorDistanceX), abs(errorDistanceY), newErrorDistance);

        RCLCPP_INFO(this->get_logger(), "Distance Error: %2f", newErrorDistance);

        if (abs(newErrorDistance) <= 0.03) {
            return true;
        }

        float speedFactor = KpForward * errorProportional + KiForward * errorIntegral + KdForward * errorDerivative;

        cmd_vel_msg.linear.x = speedFactor * 0.5;
        cmd_vel_msg.angular.z = 0.0;

        errorDistance = newErrorDistance;

        return false;
    }

    bool turnPid(std::tuple<float, float> goal, float deltaTime) {

        float errorDistanceX = std::get<0>(goal) - currentPosition.position.x;
        float errorDistanceY = std::get<1>(goal) - currentPosition.position.y;

        float theta = atan2(errorDistanceY, errorDistanceX);

        float newErrorAngle = theta - calculateYaw();
        newErrorAngle = atan2(sin(newErrorAngle), cos(newErrorAngle));

        RCLCPP_INFO(this->get_logger(), "Angle Error: %2f", newErrorAngle);

        // Error for the proportional term
        float errorProportional = newErrorAngle;

        // Error for the integral term.
        errorIntegral = errorIntegral + newErrorAngle * deltaTime;

        // Error for the derivative term.
        float errorDerivative = (newErrorAngle - errorAngle) / deltaTime;

        if (abs(newErrorAngle) < 0.01) {
            return true;
        }

        float speedFactor = KpTurn * errorProportional + KiTurn * errorIntegral + KdTurn * errorDerivative;

        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = speedFactor * 1.0;

        errorAngle = newErrorAngle;

        return false;
    }

    void errorReset() {
        errorIntegral = 0.0;
        errorDistance = 0.0;
        errorAngle = 0.0;
    }

    float calculateYaw() {
        tf2::Quaternion q(
            currentPosition.orientation.x, currentPosition.orientation.y,
            currentPosition.orientation.z, currentPosition.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        return yaw;
    }

    // PID gains
    float KpForward = 1.5;
    float KiForward = 0.1;
    float KdForward = 0.5;

    float KpTurn = 1.5;
    float KiTurn = 0.01;
    float KdTurn = 2.0;

    MoveType moveType = MoveType::FORWARD;
    MoveType nextMoveType = MoveType::TURN;

    geometry_msgs::msg::Pose currentPosition;

    std::chrono::time_point<std::chrono::system_clock> lastExecutionTime;

    size_t goalCounter = 0;
    std::vector<std::tuple<float, float>> goals = {{0.45, 0.0}, {0.50, -1.3}, {1.1, -1.3}, {1.1, -0.85}, {1.5, -0.85}, 
        {1.5, -0.3}, {2.0, -0.3}, {2.0, 0.6}, {1.5, 0.6}, {1.5, 0.25}, {0.9, 0.25}, {0.7, 0.6}, {0.2, 0.6}};

    float errorIntegral;
    float errorDistance;
    float errorAngle;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    geometry_msgs::msg::Twist cmd_vel_msg;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidMazeSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
