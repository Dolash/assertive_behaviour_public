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
#include <p2os_driver/GripperState.h>
#include <std_msgs/Float32.h>
#include <p2os_driver/SonarArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>
#include <autonomy_leds_msgs/LED.h>
#include <autonomy_leds_msgs/Keyframe.h>
//#include <vector>

#define PI 3.14159
#define TWO_PI 6.283185*/

class AssertiveBehaviour {
private:

	ros::Time timer;
    bool panicking;
    bool fighting;
    bool navigating;
    bool driving;
    /*This stores the latest laser scan's 180 ranges*/
	std::vector<float> latestLaserScan;
	/*This stores the latest sonar scan's 16 ranges*/
	std::vector<double> latestSonarScan;
	/*These bools confirm that the laser and sonar are transmitting, so that they won't be used unless these are triggered.*/
    bool laserReceived;
    bool sonarReceived;
    /*This stores the results of the latest leg detection sweep*/
    geometry_msgs::PoseArray latestLegPoseArray;
    geometry_msgs::TransformStamped latestPoses;
    geometry_msgs::TransformStamped latestSubjectPoses;
    bool legReceived;
    /*This bool reports whether legs have been detected close to the front arc, risking collision.*/
    bool legWarning;
    
    /*These bools are set according to rear sonar readings and used when deciding how to back away.*/
    bool behindRightClear;
    bool behindLeftClear;
    bool behindMiddleClear;
    
    /*This bool confirms the vicon is transmitting poses correctly*/
    bool poseReceived;
    bool viconMode;
    
	/*Controls the loop rate*/
 	double loopHz;
 	
 	bool brave;
 	
 	
 	bool returnTrip;
 	
    float startX;
    float startY;

    float goalX;
    float goalY;
        
    bool subjectDetected;
    
    float initialX;
    float initialY;
    
    bool defeat;	
    float aggression;
    
    
    /*Set these to whichever sound_player values for different sounds you want and they play at the appropriate times*/
    int startupSound;
    int fightStartSound;
    int loseFightSound;
    int winFightSound;
    int backToNormalSound;
    /*Set these to whichever setLights values for different light arrangements you want and they play at the appropriate times*/
    int startupLights;
    int fightStartLights;
    int loseFightLights;
    int winFightLights;
    int backToNormalLights;
    

  	/*This callback is for the leg detector using laser data*/
  	void legCallback(const geometry_msgs::PoseArray legData);
  	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
  	void viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
  	void laserCallback(const sensor_msgs::LaserScan scanData);
  	void sonarCallback(const p2os_driver::SonarArray sonarData);
	
	void navigatingBehaviour();
	void fightingBehaviour();
	void panickingBehaviour();
	void openGripper();
	void closeGripper();
	//void legAhead();
	void waypointing();
	float getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn);
	void reverseClearance();
	void viconSubjectAhead();
	float obstacleAvoider(float desired);
	void subjectAhead();
	void setLights(int setting);

protected:
  ros::NodeHandle nh;
  ros::NodeHandle privNh;
  /*Publisher connected to pioneer's cmd_vel topic*/
	ros::Publisher cmd_vel_pub;
	/*Publisher to control gripper*/
	ros::Publisher gripper_pub;
	/*Publisher that says what sounds to play*/
	ros::Publisher audio_pub;
	/*Publisher to set the LEDs manually*/
	ros::Publisher led_pub;
	/*Publisher to set the LEDs via keyframe*/
	ros::Publisher keyframe_pub;
	
	/*Movement orders for Pioneer*/
	geometry_msgs::Twist move_cmd;
    /*An audio request*/
    std_msgs::UInt16 audio_cmd;
    /*A gripper command*/
    p2os_driver::GripperState grip_cmd;
    /*An LED command*/
	autonomy_leds_msgs::LED led_cmd;
	/*A single keyframe which can be used to set the leds different colours. Kept simple instead of animated for now.*/
    autonomy_leds_msgs::Keyframe lights;
	
 	/*Subscriber for the leg detection from laser scans*/
 	ros::Subscriber legSub;
 	/*Subscriber for vicon-derived pose*/
 	ros::Subscriber poseSub;
 	/*Subscriber for vicon-derived subject's pose*/
 	ros::Subscriber subjectPoseSub;
 	/*Subscriber for filtered laser data*/
 	ros::Subscriber laserSub;
 	/*Subscriber for raw sonar data*/
 	ros::Subscriber sonarSub;
 	
public:
  AssertiveBehaviour(ros::NodeHandle& nh);
  ~AssertiveBehaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class AssertiveBehaviour

#endif // ASSERTIVE_BEHAVIOUR_H
