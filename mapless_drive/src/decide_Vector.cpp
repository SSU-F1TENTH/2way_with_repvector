// ssuwoong team code

#include <iostream>
#include <cmath>
#include <vector>
// #include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace sensor_msgs::msg;
using namespace ackermann_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;
using namespace visualization_msgs::msg;
using namespace std;
using namespace std::chrono_literals;

using std::placeholders::_1;

struct Vector2D
{
    double x;
    double y;

    // Vector2D(double x_, double y_) : x(x_), y(y_) {}
};

// std::ofstream debugging_os;

LaserScan::SharedPtr scan_data;
int max_gap_index = 0;

class VectorPublisher : public rclcpp::Node
{
public:
    VectorPublisher()
        : Node("vector_publisher")
    {
        // debugging_os.open("/home/user/bag_ws/bgkim/test1.csv");
        drive_publisher_ = this->create_publisher<AckermannDriveStamped>("/drive", 10);
        lidar_subscriber_ = this->create_subscription<LaserScan>("/scan", 10, std::bind(&VectorPublisher::lidar_callback, this, _1));
        // odom_subscriber_ = this->create_subscription<Odometry>("/ego_racecar/odom", 10, std::bind(&VectorPublisher::odom_callback, this, _1));
        vis_publisher_ftg = this->create_publisher<Marker>("/visualization_marker_ftg", 10);
        vis_publisher_obs = this->create_publisher<Marker>("/visualization_marker_obs", 10);
        vis_publisher_calc = this->create_publisher<Marker>("/visualization_marker_calc", 10);
    }
    // VectorPublisher::~VectorPublisher()
    // {
    //     debugging_os.close();
    // }
private:
    void lidar_callback(const LaserScan::SharedPtr msg) const
    {
        scan_data = msg;

        Vector2D A;
        Vector2D B;
        Vector2D C;

        // const value--------------------------------------------------------------
        double k_1 = 0.2; // weight for vector1(1 - k_1) vector2(k_1)
        double k_2 = 0.2; // weight for vector2(1 - k_2) vector3(k_2)
        double k_3 = 0.4; // weight for speed // angle(1 - k_3) dist(k_3)
        double k_4 = 0.3; // weight for acceleration // angle(1 - k_4) dist(k_4)

        double max_speed = 6.0;
        double min_speed = 0.4;
        double max_acceleration =1.0;

        int gap_size = 151; // using for cal vector1
        int max_section = 10; // using for cal vector2
        int front_gap = 100; // using for cal vector3 
        // const value--------------------------------------------------------------


        AckermannDriveStamped message;

        float range_max = msg->range_max;
        float angle_min = msg->angle_min;

        float angle_increment = msg->angle_increment;
        std::vector<float> ranges = msg->ranges;
        auto vector_1 = get_vector(gap_size);
        display_gap(gap_size);

        auto vector_2 = get_vector2(ranges, gap_size, max_section);
        display_gap_2(vector_2);

        Vector2D tmp_vector = calculate_vector(A, B, k_1);

        // get_vector3 == 반발 벡터를 구하는 함수
        auto vector_3 = get_vector3(ranges, angle_increment, angle_min, tmp_vector, front_gap); 
        
        A.x = vector_1.x;
        A.y = vector_1.y;
        B.x = vector_2.x;
        B.y = vector_2.y;
        C.x = vector_3.x;
        C.y = vector_3.y;


        Vector2D result_vector = calculate_vector(tmp_vector, C, k_2);
        display_gap_3(result_vector);
        // result_vector는 vector1,2,3을 각각 가중치를 곱해 더한값

        double goal_dist = result_vector.x;
        double steering_angle = result_vector.y;

        // 속도와 가속도는 result_vector의 결과를 근거로 판단한다.
        double final_speed = speed_control_mod(steering_angle, max_speed, min_speed, goal_dist, k_3);
        double final_acceleration = acceleration_control_mod(steering_angle, max_acceleration, goal_dist, k_4);


        message.drive.speed = final_speed;
        message.drive.acceleration = final_acceleration;
        message.drive.steering_angle = steering_angle;
        // debugging_os <<  timestamp <<", " << steering_angle << '/n';
        std::cout << "speed: " << final_speed << "   acceleration: " << final_acceleration << "   steer_angle" << steering_angle << std::endl;
        std::cout << "k_1: " << k_1 <<  " k_2: " << k_2 << std::endl;
        std::cout << "range_max: " << range_max <<  " angle_min " << angle_min << " angle_increasement" << angle_increment << std::endl;
        // divide_range(ranges, angle_increment, angle_min);
        message.drive.steering_angle_velocity = 0.05;
        drive_publisher_->publish(message);
    }

    vector<float> trim_filter_ranges() const
    {
        auto ranges = scan_data->ranges;

        auto last_index = int(ranges.size()-1);
        float distance = 0;

        // INFINITY의 값을 lidar값의 최대값인 200으로 설정 
        float max_distance = 200;

        for(int i = 0; i<=last_index; i++){
            distance = ranges.at(i);

            if(distance > max_distance)
                ranges.at(i) = max_distance;
            else if(distance == INFINITY)
                ranges.at(i) = max_distance;
            else
                continue;
        }

        return ranges;
    }

    
    // // 장애물과의 반발벡터를 구하는 함수
   

    // 장애물과의 반발벡터를 구하는 함수
    Vector2D get_vector3(vector<float> &ranges, float angle_increment, float angle_min, Vector2D vec, int front_gap) const
    {
        //auto ranges = trim_filter_ranges();

        Vector2D xy_point_sum;
        double x_point_sum;
        double y_point_sum;
        // only radian calculate.
        double rad_x_sum;
        double rad_y_sum;


        int ranges_size = ranges.size();
        int gap_size = 10;
        int total_points_num = ranges_size / gap_size;

        // ------------------------------------------------------------------------------------------
        Vector2D goal_vector;
        goal_vector.x = vec.x;
        goal_vector.y = vec.y;

        float goal_dist = goal_vector.x;
        float goal_angle = goal_vector.y;

        //전방의 정의: 반발벡터를 구하지 않는 ranges의 범위
        // 전방의 크기는 goal까지의 거리에 비례한다
        //float front_size = 125.0f;
        float front_size = (float) (front_gap * (goal_dist / 200.0)); // const will be changed 
        float gap_angle = front_size * angle_increment;
        //float gap_angle = 2.0f * (goal_dist / 200.0f);

        float min_gap_angle = goal_angle - gap_angle;
        float max_gap_angle = goal_angle + gap_angle;

        min_gap_angle = (min_gap_angle < -2.35) ? -2.35 : min_gap_angle;
        max_gap_angle = (max_gap_angle > 2.35) ? 2.35 : max_gap_angle;
        //----------------------------------------------------------------------------------------------

        for (int i = 0; i < ranges_size; i++)
        {
            float cur_dist = ranges[i];
            float cur_angle = (angle_min) + (i * (angle_increment));

            if (cur_angle > max_gap_angle || cur_angle < min_gap_angle) {
                double rad_x = cos(cur_angle);
                double rad_y = sin(cur_angle);

                double cur_x_point = rad_x * ranges[i];
                double cur_y_point = rad_y * ranges[i];

                x_point_sum += cur_x_point;
                y_point_sum += cur_y_point;

                rad_x_sum = rad_x_sum + rad_x / (ranges[i]);
                rad_y_sum = rad_y_sum + rad_y / (ranges[i]);
            }
            else {
                
            }

        }

        rad_x_sum = rad_x_sum / total_points_num;
        rad_y_sum = rad_y_sum / total_points_num;
        x_point_sum = x_point_sum / total_points_num;
        y_point_sum = y_point_sum / total_points_num;

        xy_point_sum.x = sqrt(x_point_sum * x_point_sum + y_point_sum * y_point_sum);
        xy_point_sum.y = atan2(-rad_y_sum, rad_x_sum);

        return xy_point_sum;
    }
    
    // goal에서 장애물을 피한 제2의 goal을 선정하는 함수
    Vector2D get_vector2(vector<float> &ranges, int gap_size, int max_section) const
    {
        //auto ranges = trim_filter_ranges();

        int ranges_size = ranges.size();
        int gap = 5;
        //int total_points_num = ranges_size / gap;

        double gradient;
        int section_cnt = 1;
        //int max_section = 10;

        double section[max_section] = { (double) (ranges_size - 1), };
        section[0] = 0;
        
        for (int i = 0; i < ranges_size - 1; i++)
        {
            // double cur_angle = (angle_min) + (i*(angle_increment));
            // double next_angle = cur_angle+gap_dize;
            
            if (ranges[i] > ranges[i + 1])
                gradient = ranges[i] / ranges[i + 1];
            else
                gradient = ranges[i + 1] / ranges[i];

            if (gradient > 3.0)
            {
                if (section[section_cnt - 1] + gap < i)
                {
                    section[section_cnt] = i;
                    section_cnt++;
                    section_cnt = min(section_cnt, 10);
                }
            }
        }

        double x = 0; // initial of x
        double y = 0; // initial of y
        Vector2D coordinate;

        auto last_index = int(ranges.size() - 1);

        float max_gap = 0;

        for (int i = 0; i <= last_index - (gap_size - 1); i++)
        {
            float temp_sum = 0;
            for (int j = i; j <= i + (gap_size - 1); j++)
            {
                temp_sum = temp_sum + ranges.at(j);
            }

            if (temp_sum > max_gap)
            {
                max_gap = temp_sum;
                max_gap_index = i + int(floor(gap_size / 2));
            }
            else
            {
                continue;
            }
        }
        int section_min_idx = 0;
        int section_max_idx = last_index;
        for (int i = 0; i < section_cnt - 1; i++)
        {
            if (section[i + 1] > max_gap_index)
            {
                section_min_idx = section[i];
                section_max_idx = section[i + 1];
                break;
            }
        }
        // std::cout << "section_min : " << section_min_idx << "section_max : " <<section_max_idx << std::endl;
        // std::cout << "max_gap_index : "<<max_gap_index<<std::endl;
        std::cout << "section count : " << section_cnt << std::endl;
        max_gap_index = floor((section_min_idx + section_max_idx) / 2);

        float rads = rads_to_follow(last_index, max_gap_index);
        if (abs(rads) > 0.698131701)
        {
            rads = rads / abs(rads) * 0.698131701;
        }

        // change radian to degree.
        double angle = to_degrees(rads);

        // get range of the way point.
        double rads_to_index = rads;
        if (rads_to_index > 0)
        {
            rads_to_index = scan_data->angle_max - rads_to_index;
        }
        int i = (rads_to_index - scan_data->angle_min) / (scan_data->angle_increment);
        double length = scan_data->ranges[i];

        if (angle > 0)
        {
            x = -(length * sin(rads));
            y = (length * cos(rads));
        }
        else if (angle < 0)
        {
            x = length * sin(abs(rads));
            y = length * cos(abs(rads));
        }
        else
        { // This is for 0 angle.
            y = length;
        }

        coordinate.x = sqrt(y * y + x * x);
        coordinate.y = rads;

        return coordinate;
    }

    // goal을 선정하는 함수
    Vector2D get_vector(int gap_size) const
    {
        auto ranges = trim_filter_ranges();
        double x = 0; // initial of x
        double y = 0; // initial of y
        Vector2D coordinate;

        auto last_index = int(ranges.size() - 1);

        float max_gap = 0;

        for (int i = 0; i <= last_index - (gap_size - 1); i++)
        {
            float temp_sum = 0;
            for (int j = i; j <= i + (gap_size - 1); j++)
            {
                temp_sum = temp_sum + ranges.at(j);
            }

            if (temp_sum > max_gap)
            {
                max_gap = temp_sum;
                max_gap_index = i + int(floor(gap_size / 2));
            }
            else
                continue;
        }

        float rads = rads_to_follow(last_index, max_gap_index);
        if (abs(rads) > 0.698131701)
        {
            rads = rads / abs(rads) * 0.698131701;
        }

        // change radian to degree.
        double angle = to_degrees(rads);

        // get range of the way point.
        double rads_to_index = rads;
        if (rads_to_index > 0)
        {
            rads_to_index = scan_data->angle_max - rads_to_index;
        }
        int i = (rads_to_index - scan_data->angle_min) / (scan_data->angle_increment);
        double length = scan_data->ranges[i];

        if (angle > 0)
        {
            x = -(length * sin(rads));
            y = length * cos(rads);
        }
        else if (angle < 0)
        {
            x = length * sin(abs(rads));
            y = length * cos(abs(rads));
        }
        else
        { // This is for 0 angle.
            y = length;
        }

        coordinate.x = sqrt(y * y + x * x);
        coordinate.y = rads;

        return coordinate;
    }

    // 속도와 가속도는 result_vector의 결과를 근거로 판단한다.
    double speed_control_mod(double radian, double max_speed, double min_speed, double goal_dist, double k_3) const
    {
        double degree = to_degrees(radian);
        if (abs(degree) > 12)
        {
            degree = 12;
        } // 12 is the specific angle for maximum.
        // double input_spe ed = (max_speed - (max_speed - min_speed) * abs(degree) / (12));
        double input_speed = (max_speed - ((max_speed - min_speed) * ((abs(degree) / (12)) * (1 - k_3)) + ((1 - goal_dist / 200) * (k_3))));
        if (input_speed > max_speed)
        {
            return max_speed;
        }
        if (input_speed < min_speed)
        {
            return min_speed;
        }
        return input_speed;
    }

    // 속도와 가속도는 result_vector의 결과를 근거로 판단한다.
    double acceleration_control_mod(double radian, double max_acceleration, double goal_dist, double k_4) const
    {
        double degree = to_degrees(radian);
        if (abs(degree) > 12)
        {
            degree = 12;
        } // 12 is the specific angle for maximum.
        double input_acceleration = (max_acceleration - (max_acceleration * ((abs(degree) / (12)) * (1 - k_4)) + ((1 - goal_dist / 200) * k_4)));
        if (input_acceleration > max_acceleration)
        {
            return max_acceleration;
        }
        return input_acceleration;
    }


    // vector A와 B를 가중치 를 k로 두고 더하는 함수
    Vector2D calculate_vector(const Vector2D &A, const Vector2D &B, double k) const {
        Vector2D result_vector;
        
        double k_a = 1.0 - k;
        double k_b = k;

        double result_x = ((k_a * A.x) + (k_b*B.x));
        double result_y = ((k_a * A.y) + (k_b*B.y));
        result_vector.x = result_x;
        result_vector.y = result_y;
        //display_gap_3(result_vector);
        
        return result_vector;
    }

    //-------------------------display_part----------------------------------------------------------------------------------------------
    void display_gap(int gap_size) const
    {
        auto vector_1 = get_vector(gap_size);

        // float rads = rads_to_follow(last_index, max_gap_index);
        // float point_location = tan(rads) * 1;

        Marker marker;
        marker.header.frame_id = "ego_racecar/base_link";
        marker.header.stamp = now();
        marker.ns = "gaps";
        marker.id = 0;
        marker.type = Marker::POINTS;
        marker.action = Marker::ADD;
        marker.frame_locked = true;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        Point p;
        p.x = vector_1.x * cos(vector_1.y) / 2;
        p.y = vector_1.x * sin(vector_1.y) / 2;
        p.z = 0.05;

        marker.pose.position = p;
        marker.points.push_back(p);
        // std::cout << "민혁 display 좌표: " << p.x << " " << p.y << std::endl;
        vis_publisher_ftg->publish(marker);
    }

    void display_gap_2(Vector2D vector_2) const
    {
        Marker marker1;
        marker1.header.frame_id = "ego_racecar/base_link";
        marker1.header.stamp = now();
        marker1.ns = "gaps_min";
        marker1.id = 0;
        marker1.type = Marker::POINTS;
        marker1.action = Marker::ADD;
        marker1.frame_locked = true;
        marker1.scale.x = 0.2;
        marker1.scale.y = 0.2;

        marker1.color.r = 0.0f;
        marker1.color.g = 1.0f;
        marker1.color.b = 0.0f;
        marker1.color.a = 3.0;

        Point p;
        p.x = -vector_2.x * cos(vector_2.y) / 2;
        p.y = vector_2.x * sin(vector_2.y) / 2;
        p.z = 0.05;

        marker1.pose.position = p;
        marker1.points.push_back(p);

        // std::cout << "민 : " << marker1.pose.position.x << " " << marker1.pose.position.y << std::endl;

        vis_publisher_obs->publish(marker1);
    }

    void display_gap_3(Vector2D vector_3) const
    {

        Marker marker;
        marker.header.frame_id = "ego_racecar/base_link";
        marker.header.stamp = now();
        marker.ns = "gaps_min";
        marker.id = 0;
        marker.type = Marker::POINTS;
        marker.action = Marker::ADD;
        marker.frame_locked = true;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 3.0;

        Point p;
        p.x = vector_3.x * cos(vector_3.y) / 2;
        p.y = vector_3.x * sin(vector_3.y) / 2;
        p.z = 0.05;

        marker.pose.position = p;
        marker.points.push_back(p);

        // std::cout << "민 : " << marker1.pose.position.x << " " << marker1.pose.position.y << std::endl;

        vis_publisher_calc->publish(marker);
    }
    //-------------------------display_part----------------------------------------------------------------------------------------------

    double rads_to_follow(int last_index, int max_gap_index) const
    {
        double angle_increment = 0.004351851996034384;
        int relative_point = max_gap_index - int(floor((last_index + 1) / 2));
        float rads = angle_increment * relative_point;

        return rads;
    }

    void follow_gap(int last_index, int max_gap_index) const
    {
        float rads = rads_to_follow(last_index, max_gap_index);

        if (abs(rads) > 0.698131701)
        {
            rads = rads / abs(rads) * 0.698131701;
        }
    }

    double vectorAngle(const Vector2D &vector) const
    {
        return atan2(vector.y, vector.x);
    }

    double to_radians(double theta) const
    {
        return M_PI * theta / 180.0;
    }

    double to_degrees(float theta) const
    {
        return theta * 180.0 / M_PI;
    }

    double vectorLength(const Vector2D &vector) const
    {
        return sqrt(vector.x * vector.x + vector.y * vector.y);
    }

    rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<Marker>::SharedPtr vis_publisher_ftg;
    rclcpp::Publisher<Marker>::SharedPtr vis_publisher_obs;
    rclcpp::Publisher<Marker>::SharedPtr vis_publisher_calc;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorPublisher>());
    rclcpp::shutdown();
    return 0;
}