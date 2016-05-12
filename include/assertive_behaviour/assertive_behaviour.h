#ifndef ASSERTIVE_BEHAVIOUR_H
#define ASSERTIVE_BEHAVIOUR_H

#include <ros/ros.h>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <p2os_driver/GripperState.h>
#include <std_msgs/UInt16.h>
//#include <vector>

#define PI 3.14159
#define TWO_PI 6.283185*/

class AssertiveBehaviour {
private:

	ros::Time cycleStartTime;
    bool panicking;
    bool fighting;
    bool navigating;
    bool driving;
    /*This stores the latest laser scan's 180 ranges*/
	//std::vector<float> latestLaserScan;
    //bool laserReceived;
    /*This stores the results of the latest leg detection sweep*/
    geometry_msgs::PoseArray latestLegPoseArray;
    geometry_msgs::PoseArray latestPoses;
    bool legReceived;
    /*This bool reports whether legs have been detected close to the front arc, risking collision.*/
    bool legWarning;
    
    bool poseReceived;
    
    bool returnTrip;
	
	/*Controls the loop rate*/
 	double loopHz;
 	
	/*The robot's yaw, extracted from the localization subscription*/
 	//double yaw;
	/*These two check that we're receiving poses and charge readings from the two subscriptions, the loop doesn't work without them*/
 	//bool poseReceived;
	//bool chargeReceived;
	/*Used while homing in on recharge location, to differentiate between when you stop to turn toward the target and start driving forward*/
	//bool driving;

	/*parameters that control when the robot goes to recharge if using battery level information*/
	//double chargeLevel;

	//double chargeLatest;
	//double highThreshold;
	//double midThreshold;
	//double lowThreshold;

	/*state variables*/
	//bool recharging;
	//int chargeState;
	//int buoyPresence;

	/*parameters that name the robot, and specify where its charger is in the world*/
	//double chargerX;
	//double chargerY1;
	//double chargerY2;
	//bool chargerPatrolReset;
	//std::string robotName;

	/*parameters that control when the robot goes to recharge if using time*/

	//bool chargeTime;
	//bool backupTimeCheck;
	//double highTime;
	//double midTime;
	//double lowTime;
	//double highChargeTime;
	//double midChargeTime;
	//double lowChargeTime;


    /*This callback is for the laser*/
  	//void laserCallback(const sensor_msgs::LaserScan scanData);
  	/*This callback is for the leg detector using laser data*/
  	void legCallback(const geometry_msgs::PoseArray legData);
  	void viconCallback(const geometry_msgs::PoseArray poseData);
  	
	//void chargeLevelCallback(const std_msgs::Float32 charge);
	//void buoyCallback(const std_msgs::UInt16 irReading);
	//float getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn);
	//void approachCharger();
	//void whileActive();
	//void whileRecharging();
	void navigatingBehaviour();
	void fightingBehaviour();
	void panickingBehaviour();
	void openGripper();
	void closeGripper();
	void legAhead();
	void waypointing();
	float getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn);

protected:
  ros::NodeHandle nh;
  ros::NodeHandle privNh;
  /*Publisher connected to pioneer's cmd_vel topic*/
	ros::Publisher cmd_vel_pub;
	ros::Publisher gripper_pub;
    /*Subscriber for the laser*/
 	//ros::Subscriber laserSub;
 	/*Subscriber for the leg detection from laser scans*/
 	ros::Subscriber legSub;
 	ros::Subscriber poseSub;
 	/*Movement orders for Pioneer*/
	geometry_msgs::Twist move_cmd;
	
	
	/*ros::Subscriber chargeLevelSub;
	ros::Subscriber buoySub;
    */
    
    
	
	
	/*ros::Publisher dock_pub;
	ros::Publisher undock_pub;

 	geometry_msgs::TransformStamped ownPose;
*/
	


public:
  AssertiveBehaviour(ros::NodeHandle& nh);
  ~AssertiveBehaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class AssertiveBehaviour

#endif // ASSERTIVE_BEHAVIOUR_H
