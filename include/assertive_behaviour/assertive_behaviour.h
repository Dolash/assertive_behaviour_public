#ifndef ASSERTIVE_BEHAVIOUR_H
#define ASSERTIVE_BEHAVIOUR_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
/*#include <geometry_msgs/Pose.h>*/
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
/*#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>*/
#include <std_msgs/Float32.h>
//#include <std_msgs/UInt16.h>
//#include <vector>

#define PI 3.14159
#define TWO_PI 6.283185*/

class AssertiveBehaviour {
private:

	ros::Time cycleStartTime;
    bool panicking;
    bool fighting;
    bool navigating;
	std::vector<float> latestLaserScan;
    bool laserReceived;
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

  	void laserCallback(const sensor_msgs::LaserScan scanData);
	//void chargeLevelCallback(const std_msgs::Float32 charge);
	//void buoyCallback(const std_msgs::UInt16 irReading);
	//float getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn);
	//void approachCharger();
	//void whileActive();
	//void whileRecharging();
	void navigatingBehaviour();
	void fightingBehaviour();
	void panickingBehaviour();

protected:
  ros::NodeHandle nh;
  ros::NodeHandle privNh;
  
 	ros::Subscriber laserSub;
	/*ros::Subscriber chargeLevelSub;
	ros::Subscriber buoySub;
    */
    
    /*Publisher connected to pioneer's cmd_vel topic*/
	ros::Publisher cmd_vel_pub;
	
	
	/*ros::Publisher dock_pub;
	ros::Publisher undock_pub;

 	geometry_msgs::TransformStamped ownPose;
*/
	/*Movement orders for Pioneer*/
	geometry_msgs::Twist move_cmd;


public:
  AssertiveBehaviour(ros::NodeHandle& nh);
  ~AssertiveBehaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class AssertiveBehaviour

#endif // ASSERTIVE_BEHAVIOUR_H
