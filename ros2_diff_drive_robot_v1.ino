
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "MPU9250.h"
MPU9250 mpu;

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
//MMM
#include <geometry_msgs/msg/twist.h>
//MMM 11-19-2021
#include <std_msgs/msg/header.h> //comes with imu sensor
//#include <std_msgs/msg/string.h> //comes with imu sensor

//#include <rosidl_runtime_c/string.h>
//#include <rosidl_runtime_c/string_functions.h>

#include <sensor_msgs/msg/imu.h>  //comes with imu sensor
#include <geometry_msgs/msg/quaternion_stamped.h> //comes with imu sensor
#include <geometry_msgs/msg/vector3_stamped.h> //comes with imu sensor 
#include <geometry_msgs/msg/vector3.h> //comes with imu sensor 
#include <nav_msgs/msg/odometry.h> //for publishing odom msgs



//MMM
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;


rcl_publisher_t publisher;  //this publisher is for /tf
rcl_publisher_t publisher_i;  //this publisher is for /imu
rcl_publisher_t publisher_nav;  //this publisher is for /odom
tf2_msgs__msg__TFMessage * tf_message;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
//MMM 11-19-2021
sensor_msgs__msg__Imu * imu; //adding to publish imu sensor data
nav_msgs__msg__Odometry  *odometry; //this is for odom


//MMM
#define PWM_MIN 0
#define PWMRANGE 255

// Declare functions
void setupPins();
float mapPwm(float x, float out_min, float out_max);

//Pins for motors
#define left_back 3
#define left_front 4
#define right_front 5
#define right_back 6
#define nD2 7
#define nSF 8

const int R = 0.061;  //Radius of wheel in meter
const int L = 0.235;  //Length of wheel base in meter



#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);
//cIMU    IMU;


//MMM----------------------------------------------------
//-------------velocity and position valiables-----------

float velocity_x = 0.0;
float velocity_y = 0.0;

float position_x = 0.0;
float position_y = 0.0;
float position_z = 0.0;
float angular_velocity_z = 0.0;
float current_time = 0.0;
float last_time = 0.0;
float heading_ = 0.0;


//double deltaTime = 0.0;
unsigned long prev_odom_update = 0;

//MMM this function will calculate the position and velocity from imu data 
//since we do not have a encoder for the wheel.
void get_pos_vel_for_odom(){
  //struct timespec tv = {0};
  //clock_gettime(0, &tv);
  //current_time = tv.tv_sec;
  //float deltaTime = (current_time - last_time)/ 1000.0; // delta time to find the position and velocity
  //last_time = current_time;
  //float deltaTime = 1/46.0;

  
  double ax = mpu.getLinearAccX();
  double ay = mpu.getLinearAccY();

  //calculating average of the accel_x
  float avg_ax = 0.0;
    for (int i=0; i < 100; i++) {
        avg_ax = avg_ax + ax;
    }
    avg_ax = avg_ax/100.0;

  //calculating average of the accel_y
  float avg_ay = 0.0;
    for (int i=0; i < 100; i++) {
        avg_ay = avg_ay+ ay;
    }
    avg_ay = avg_ay/100.0;


  unsigned long now = millis();
  float deltaTime = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;

  angular_velocity_z = (double) mpu.getGyroZ();
  float temp_heading  = (float) mpu.getMagY();

  //Calculating velocity

  
  velocity_x += deltaTime * avg_ay; //(float) mpu.getLinearAccY(); // calculating velocity from imu data
  velocity_y += deltaTime * -avg_ax; //-(float) mpu.getLinearAccX(); // calculating velocity from imu data
  //velocity_z = deltaTime * (double) mpu.getAccZ(); // calculating velocity from imu data

  //velocity_x_initial = velocity_x;
  //velocity_y_initial = velocity_y;
  
  //calculating position
  //delta_position_x = deltaTime * velocity_x; // calculating position from imu data
  //delta_position_y = deltaTime * velocity_y; // calculating position from imu data
  //delta_position_z = deltaTime * velocity_z;  // calculating position from imu data

  
  float delta_heading = angular_velocity_z * deltaTime; //radians
  float cos_h = cos(heading_);
  float sin_h = sin(heading_);
  float delta_x = (velocity_x * cos_h - velocity_y * sin_h) * deltaTime; //m
  float delta_y = (velocity_x * sin_h + velocity_y * cos_h) * deltaTime; //m

  //calculate current position of the robot
  position_x += delta_x;
  position_y += delta_y;
  heading_   += delta_heading;


  //calculate current position of the robot
  //position_x = velocity_x * deltaTime + (0.5 * avg_ay) * deltaTime * deltaTime;
  //position_y = velocity_y * deltaTime + (0.5 * avg_ax) * deltaTime * deltaTime;


  //if (avg_ax <= 0.10) {
  //  velocity_y = 0.0;
  //  position_x = 0.0;
    
  //}

  //if (avg_ay <= 0.32){
  //  velocity_x = 0.0;
  //  position_y = 0.0;
  //}
  

  
  //calculate robot's heading in quaternion angle
  //ROS has a function to calculate yaw in quaternion angle
  float q[4];
  euler_to_quat(0, 0, heading_, q);

    
  
}


//Calculate the quartenion from the angles
const void euler_to_quat(float roll, float pitch, float yaw, float* q) 
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}


//MMM--------------------------------
void setupPins()
{
  // Status motor
  pinMode(LED_PIN, OUTPUT);  
  pinMode(left_back, OUTPUT);
  pinMode(left_front, OUTPUT);
  pinMode(right_front, OUTPUT);
  pinMode(right_back, OUTPUT);
  pinMode(nD2, OUTPUT);
  digitalWrite(nD2, HIGH); // default to on
  pinMode(nSF, INPUT);
  stop();
}

void stop()
{
  analogWrite(left_back, 0);
  analogWrite(left_front, 0);
  analogWrite(right_front, 0);
  analogWrite(right_back, 0);
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);

  // Cap values at [-1 .. 1]
  float x = max(min(msg->linear.x, 1.0f), -1.0f);
  float z = max(min(msg->angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  //float l = (2*x - z*L)/2*R;
  //float r = (2*x + z*L)/2*R ; 
  
  float l = (msg->linear.x - msg->angular.z) / 2;
  float r = (msg->linear.x + msg->angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM

  if (l < 0){
    analogWrite(left_front, 0);
    analogWrite(left_back, lPwm);
  }
  else if (l > 0){
    analogWrite(left_back, 0);
    analogWrite(left_front, lPwm);
  }
  if (r < 0){
    analogWrite(right_front, 0);
    analogWrite(right_back, rPwm);
  }
  else if (r > 0){
    analogWrite(right_back, 0);
    analogWrite(right_front, rPwm);
  }
  else{
    stop();
  }
  
  
  //analogWrite(right_front, lPwm);
  //analogWrite(right_back, rPwm);
  //analogWrite(left_front, lPwm);
  //analogWrite(left_back, rPwm);
}


// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

//-------------------------------------

void setup() {
  set_microros_transports();
  setupPins();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 
  
  Wire.begin();

  //MMM
  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }


  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //MMM-----------------------------------------
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    //"micro_ros_arduino_twist_subscriber"));
    "cmd_vel"));

    //----------------------------------

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));


  //MMM 11-19-2021
  //create sensor msgs publisher
  
  RCCHECK(rclc_publisher_init_default(
    &publisher_i,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));


   //MMM 12-12-2021
  //create nav msgs publisher
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_nav,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"));
  

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  //MMM--------------------
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  //-----------------------------------------------------------------
  //--------------------------tf data--------------------------------
  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string1[] = "/map";
  memcpy(tf_message->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
  tf_message->transforms.data[0].header.frame_id.size = strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  char string2[] = "/base_link";
  tf_message->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(tf_message->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
  tf_message->transforms.data[0].child_frame_id.size = strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;
  //-----------------------------------------------------------------
  //--------------------------END of tf data--------------------------------

  //-----------------------------------------------------------------
  //--------------------------Imu data--------------------------------
  
  imu = sensor_msgs__msg__Imu__create();
  std_msgs__msg__Header__init(&imu->header);
  //geometry_msgs__msg__QuaternionStamped__Sequence__init(&imu->orientation, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->angular_velocity, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->linear_acceleration, 1);
  
  //imu->header.frame_id.data= "/imu";
  
  imu->header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string[] = "/imu";
  memcpy(imu->header.frame_id.data, string, strlen(string) + 1);
  imu->header.frame_id.size = strlen(imu->header.frame_id.data);
  imu->header.frame_id.capacity = 100;
  

  //-----------------------------------------------------------------
  //--------------------------END of imu data--------------------------------


  //-----------------------------------------------------------------
  //--------------------------Odometry data--------------------------------
  
  odometry = nav_msgs__msg__Odometry__create();
  //std_msgs__msg__Header__init(&odometry->header);
  //rosidl_runtime_c__String__init(&odometry->child_frame_id);
  //std_msgs__msg__String__init(&odometry->child_frame_id, 1);
  //geometry_msgs__msg__TransformStamped__Sequence__init(&odometry->transforms_1, 1);
  //geometry_msgs__msg__QuaternionStamped__Sequence__init(&imu->orientation, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->angular_velocity, 1);
  //geometry_msgs__msg__Vector3Stamped__Sequence__init(&imu->linear_acceleration, 1);
  
  //imu->header.frame_id.data= "/imu";
  
  odometry->header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string_odom[] = "/odom";
  memcpy(odometry->header.frame_id.data, string_odom, strlen(string_odom) + 1);
  odometry->header.frame_id.size = strlen(odometry->header.frame_id.data);
  odometry->header.frame_id.capacity = 100;

  char string_odom_2[] = "/base_footprint";
  odometry->child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(odometry->child_frame_id.data, string_odom_2, strlen(string_odom_2) + 1);
  odometry->child_frame_id.size = strlen(odometry->child_frame_id.data);
  odometry->child_frame_id.capacity = 100;
  

  //-----------------------------------------------------------------
  //--------------------------END of odometry data--------------------------------
  
  
}

//MMM Imu loop function
void imu_pub(){
  //MMM 11-19-2021
  struct timespec tv = {0};
  clock_gettime(0, &tv);
  imu->orientation.x = (double) mpu.getQuaternionY(); //q[2];
  imu->orientation.y = (double) -mpu.getQuaternionX();  //q[1];
  imu->orientation.z = (double) mpu.getQuaternionZ();  //q[3]; 
  imu->orientation.w = (double) mpu.getQuaternionW();  //q[0];

  imu->angular_velocity.x = (double) mpu.getGyroY(); 
  imu->angular_velocity.y = (double) -mpu.getGyroX(); 
  imu->angular_velocity.z = (double) mpu.getGyroZ();   

  imu->linear_acceleration.x = (double) mpu.getAccY(); 
  imu->linear_acceleration.y = (double) -mpu.getAccX(); 
  imu->linear_acceleration.z = (double) mpu.getAccZ();  
  
  imu->header.stamp.nanosec = tv.tv_nsec;
  imu->header.stamp.sec = tv.tv_sec;
}

//MMM tf loop function
void tf_pub(){
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  
  tf_message->transforms.data[0].transform.translation.x = position_x; //(double) mpu.getEulerX();//mpu.getPitch();
  tf_message->transforms.data[0].transform.translation.y = position_y; //(double) mpu.getEulerY();//mpu.getRoll();
  tf_message->transforms.data[0].transform.translation.z = position_z; //(double) mpu.getEulerZ();//mpu.getYaw();

  tf_message->transforms.data[0].transform.rotation.x = (double) mpu.getQuaternionY(); //q[2];
  tf_message->transforms.data[0].transform.rotation.y = (double) -mpu.getQuaternionX();  //q[1];
  tf_message->transforms.data[0].transform.rotation.z = (double) mpu.getQuaternionZ();  //q[3]; 
  tf_message->transforms.data[0].transform.rotation.w = (double) mpu.getQuaternionW();  //q[0];
  tf_message->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
  tf_message->transforms.data[0].header.stamp.sec = tv.tv_sec;
  
}


//MMM nav loop function
void nav_pub(){

  get_pos_vel_for_odom();
  struct timespec pv = {0};
  clock_gettime(0, &pv);
  float q[4];
  euler_to_quat(0, 0, heading_, q);
  

  odometry->header.stamp.nanosec = pv.tv_nsec;
  odometry->header.stamp.sec = pv.tv_sec;
  
  odometry->pose.pose.position.x = position_x;
  odometry->pose.pose.position.y = position_y;
  odometry->pose.pose.position.z = 0.0; //(double) position_x;
  odometry->pose.pose.orientation.x = (double) q[1]; //(double) mpu.getQuaternionY();
  odometry->pose.pose.orientation.y = (double) q[2]; //(double) -mpu.getQuaternionX();
  odometry->pose.pose.orientation.z = (double) q[3]; //(double) mpu.getQuaternionZ();
  odometry->pose.pose.orientation.w = (double) q[0]; //(double) mpu.getQuaternionW();

  odometry->pose.covariance[0] = 0.001;
  odometry->pose.covariance[7] = 0.001;
  odometry->pose.covariance[35] = 0.001;

  

  odometry->twist.twist.linear.x = velocity_x; 
  odometry->twist.twist.linear.y = velocity_y; 
  odometry->twist.twist.linear.z = 0.0; // (double) velocity_z;

  odometry->twist.twist.angular.x = 0.0; //(double) velocity_x; 
  odometry->twist.twist.angular.y = 0.0; //(double) velocity_y; 
  odometry->twist.twist.angular.z = angular_velocity_z; // (double) velocity_z;

  odometry->twist.covariance[0] = 0.001;
  odometry->twist.covariance[7] = 0.001;
  odometry->twist.covariance[35] = 0.001;
  
  
  
}

void loop() {

  //delay(100);
  
  
  //struct timespec tv = {0};
  //clock_gettime(0, &tv);

  mpu.update();
  
  imu_pub();
  nav_pub();
  tf_pub();
  
  
  RCSOFTCHECK(rcl_publish(&publisher, tf_message, NULL));
  RCSOFTCHECK(rcl_publish(&publisher_i, imu, NULL));
  RCSOFTCHECK(rcl_publish(&publisher_nav, odometry, NULL));
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  
}
