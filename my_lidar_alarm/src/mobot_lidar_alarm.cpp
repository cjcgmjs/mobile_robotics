// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <math.h>
#include <std_srvs/Trigger.h>

#define PI 3.14159265

const double MIN_SAFE_DISTANCE = 1.2; // set alarm if anything is within 0.5m of the front of robot
const double MIN_SAFE_DISTANCE_90_DEG = MIN_SAFE_DISTANCE;
const double MIN_SAFE_DISTANCE_15_DEG = MIN_SAFE_DISTANCE / cos(15 * PI / 180);
const double MIN_SAFE_DISTANCE_30_DEG = MIN_SAFE_DISTANCE / cos(30 * PI / 180);
const double MIN_SAFE_DISTANCE_45_DEG = MIN_SAFE_DISTANCE / cos(45 * PI / 180);
const double MIN_SAFE_DISTANCE_60_DEG = MIN_SAFE_DISTANCE_30_DEG;
const double MIN_SAFE_DISTANCE_75_DEG = MIN_SAFE_DISTANCE_15_DEG;
double MIN_SAFE_DISTANCE_MIN_ANGLE = 0;
double MIN_SAFE_DISTANCE_MAX_ANGLE = 0;

// these values to be set within the laser callback
float ping_dist_in_front_= 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_in_front_ = -1; // NOT real; callback will have to find this

float ping_dist_left_15_deg_ = 3.0;
int ping_index_left_15_deg_ = -1;
float ping_dist_left_30_deg_ = 3.0;
int ping_index_left_30_deg_ = -1;
float ping_dist_left_45_deg_ = 3.0;
int ping_index_left_45_deg_ = -1;
float ping_dist_left_60_deg_ = 3.0;
int ping_index_left_60_deg_ = -1;
float ping_dist_left_75_deg_ = 3.0;
int ping_index_left_75_deg_ = -1;
float ping_dist_left_90_deg_ = 3.0;
int ping_index_left_90_deg_ = -1;

float ping_dist_right_15_deg_ = 3.0;
int ping_index_right_15_deg_ = -1;
float ping_dist_right_30_deg_ = 3.0;
int ping_index_right_30_deg_ = -1;
float ping_dist_right_45_deg_ = 3.0;
int ping_index_right_45_deg_ = -1;
float ping_dist_right_60_deg_ = 3.0;
int ping_index_right_60_deg_ = -1;
float ping_dist_right_75_deg_ = 3.0;
int ping_index_right_75_deg_ = -1;
float ping_dist_right_90_deg_ = 3.0;
int ping_index_right_90_deg_ = -1;


float ping_dist_min_angle_ = 3.0;
int ping_index_min_angle_ = 0;
float ping_dist_max_angle_= 3.0;
int ping_index_max_angle_ = -1;

double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
ros::ServiceClient client1_;
ros::ServiceClient client2_;
std_srvs::Trigger srv;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_in_front_ < 0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_in_front_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ping_index_right_15_deg_ = (int) ((-0.261799 -angle_min_)/angle_increment_);
        ping_index_right_30_deg_ = (int) ((-0.5236 -angle_min_)/angle_increment_);
        ping_index_right_45_deg_ = (int) ((-0.7854 -angle_min_)/angle_increment_);
        ping_index_right_60_deg_ = (int) ((-1.0472 -angle_min_)/angle_increment_);
        ping_index_right_75_deg_ = (int) ((-1.309 -angle_min_)/angle_increment_);
        ping_index_right_90_deg_ = (int) ((-1.5708 -angle_min_)/angle_increment_);

        ping_index_left_15_deg_ = (int) ((0.261799 -angle_min_)/angle_increment_);
        ping_index_left_30_deg_ = (int) ((0.5236 -angle_min_)/angle_increment_);
        ping_index_left_45_deg_ = (int) ((0.7854 -angle_min_)/angle_increment_);
        ping_index_left_60_deg_ = (int) ((1.0472 -angle_min_)/angle_increment_);
        ping_index_left_75_deg_ = (int) ((1.309 -angle_min_)/angle_increment_);
        ping_index_left_90_deg_ = (int) ((1.5708 -angle_min_)/angle_increment_);

        ping_index_max_angle_ = (int) ((angle_max_ -angle_min_)/angle_increment_);

        MIN_SAFE_DISTANCE_MIN_ANGLE = 1 / cos(-1.5708 - angle_min_);
        MIN_SAFE_DISTANCE_MAX_ANGLE = 1 / cos(angle_max_ - 1.5708);
        ROS_INFO("LIDAR setup: ping_index_in_front = %d",ping_index_in_front_);
        
    }
    
   ping_dist_in_front_ = laser_scan.ranges[ping_index_in_front_];

   ping_dist_right_15_deg_ = laser_scan.ranges[ping_index_right_15_deg_];
   ping_dist_right_30_deg_ = laser_scan.ranges[ping_index_right_30_deg_];
   ping_dist_right_45_deg_ = laser_scan.ranges[ping_index_right_45_deg_];
   ping_dist_right_60_deg_ = laser_scan.ranges[ping_index_right_60_deg_];
   ping_dist_right_75_deg_ = laser_scan.ranges[ping_index_right_75_deg_];
   ping_dist_right_90_deg_ = laser_scan.ranges[ping_index_right_90_deg_];

   ping_dist_left_15_deg_ = laser_scan.ranges[ping_index_left_15_deg_];
   ping_dist_left_30_deg_ = laser_scan.ranges[ping_index_left_30_deg_];
   ping_dist_left_45_deg_ = laser_scan.ranges[ping_index_left_45_deg_];
   ping_dist_left_60_deg_ = laser_scan.ranges[ping_index_left_60_deg_];
   ping_dist_left_75_deg_ = laser_scan.ranges[ping_index_left_75_deg_];
   ping_dist_left_90_deg_ = laser_scan.ranges[ping_index_left_90_deg_];


   ping_dist_min_angle_ = laser_scan.ranges[ping_index_min_angle_];
   ping_dist_max_angle_ = laser_scan.ranges[ping_index_max_angle_];

   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   if (ping_dist_in_front_< MIN_SAFE_DISTANCE
           || ping_dist_left_15_deg_ < MIN_SAFE_DISTANCE_15_DEG || ping_dist_left_30_deg_ < MIN_SAFE_DISTANCE_30_DEG
           || ping_dist_left_45_deg_ < MIN_SAFE_DISTANCE_45_DEG || ping_dist_left_60_deg_ < MIN_SAFE_DISTANCE_60_DEG
           || ping_dist_left_75_deg_ < MIN_SAFE_DISTANCE_75_DEG || ping_dist_left_90_deg_ < MIN_SAFE_DISTANCE_90_DEG
           || ping_dist_right_15_deg_ < MIN_SAFE_DISTANCE_15_DEG || ping_dist_right_30_deg_ < MIN_SAFE_DISTANCE_30_DEG
           || ping_dist_right_45_deg_ < MIN_SAFE_DISTANCE_45_DEG || ping_dist_right_60_deg_ < MIN_SAFE_DISTANCE_60_DEG
           || ping_dist_right_75_deg_ < MIN_SAFE_DISTANCE_75_DEG || ping_dist_right_90_deg_ < MIN_SAFE_DISTANCE_90_DEG
            // The Robot either moves foward when no collision or rotate left until no collision, 
 	    // thus pings over 90 degrees from forward are useless and are detrimental in some corner cases,
	    // such as when the robot tries to get out a cell with walls on three sides 
//           || ping_dist_min_angle_ < MIN_SAFE_DISTANCE_MIN_ANGLE
//           || ping_dist_max_angle_ < MIN_SAFE_DISTANCE_MAX_ANGLE
           ) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
       client1_.call(srv);
   }
   else {
       laser_alarm_=false;
       client2_.call(srv);
   }
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("estop_service");  
    client1_ = client;
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Trigger>("clear_estop_service"); 
    client2_ = client2;
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

