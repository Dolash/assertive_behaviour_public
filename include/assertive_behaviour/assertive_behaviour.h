#ifndef ASSERTIVE_BEHAVIOUR_H
#define ASSERTIVE_BEHAVIOUR_H

#include <ros/ros.h>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <p2os_msgs/GripperState.h>
#include <std_msgs/Float32.h>
#include <p2os_msgs/SonarArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>
#include <autonomy_leds_msgs/Keyframe.h>
#include <vector>
#include <unistd.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <move_base_msgs/MoveBaseGoal.h>


#define PI 3.14159
#define TWO_PI 6.283185*/

class AssertiveBehaviour {
private:

	ros::Time timer;
	ros::Time moveOrderTimer;
    bool panicking;
    bool fighting;
    bool navigating;
    bool driving;


	sensor_msgs::LaserScan scrubbedScan;
    /*This stores the latest laser scan's 180 ranges*/
	std::vector<float> latestLaserScan;
	/*This stores the latest sonar scan's 16 ranges*/
	std::vector<double> latestSonarScan;
	/*These bools confirm that the laser and sonar are transmitting, so that they won't be used unless these are triggered.*/
    bool laserReceived;
    bool sonarReceived;
    geometry_msgs::TransformStamped latestPoses;
    geometry_msgs::TransformStamped latestSubjectPoses;

    
	geometry_msgs::PoseArray humanMaximaArray;


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
 	
 	bool firstTime;
 	
 	bool obstacle;
 	
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

	//move_base_msgs::MoveBaseGoal goal_cmd;
	geometry_msgs::PoseStamped goal_cmd;
    
    
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
    /*Components that will go into the keyframe that we will publish to control the lights*/
    std::vector<std_msgs::ColorRGBA> lightColours;
    std_msgs::ColorRGBA temp;
    
    bool fightStarting;


	bool humanReceived;

	bool scrubbedScanReceived;


  	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
  	void viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
  	void laserCallback(const sensor_msgs::LaserScan scanData);
  	void sonarCallback(const p2os_msgs::SonarArray sonarData);
	void cmdVelListenerCallback(const geometry_msgs::Twist navData);
	void localMaximaCallback(const geometry_msgs::PoseArray maximaData);
	void scrubbedScanCallback(const sensor_msgs::LaserScan scanData);


	void navigatingBehaviour();
	void fightingBehaviour();
	void panickingBehaviour();
	void openGripper();
	void closeGripper();
	void waypointing();
	float getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn, bool destination);
	void reverseClearance();
	void viconSubjectAhead();
	float obstacleAvoider(float desired);
	void subjectAhead();
	void setLights(int setting);
	
	void laserTest();

protected:
  ros::NodeHandle nh;
  ros::NodeHandle privNh;
  /*Publisher connected to pioneer's cmd_vel topic*/
	ros::Publisher cmd_vel_pub;
	/*Publisher to control gripper*/
	ros::Publisher gripper_pub;
	/*Publisher that says what sounds to play*/
	ros::Publisher audio_pub;
	/*Publisher to set the LEDs via keyframe*/
	ros::Publisher keyframe_pub;
	/*Publisher to set the movement goal*/
	ros::Publisher goal_pub;
	


	/*Movement orders for Pioneer*/
	geometry_msgs::Twist move_cmd;
    /*An audio request*/
    std_msgs::UInt16 audio_cmd;
    /*A gripper command*/
    p2os_msgs::GripperState grip_cmd;
	/*A single keyframe which can be used to set the leds different colours. Kept simple instead of animated for now.*/
    autonomy_leds_msgs::Keyframe lights;
	
 
 	/*Subscriber for vicon-derived pose*/
 	ros::Subscriber poseSub;
 	/*Subscriber for vicon-derived subject's pose*/
 	ros::Subscriber subjectPoseSub;
 	/*Subscriber for filtered laser data*/
 	ros::Subscriber laserSub;
 	/*Subscriber for raw sonar data*/
 	ros::Subscriber sonarSub;


	ros::Subscriber humanMaximaSub;

	ros::Subscriber cmdVelListenerSub;
	ros::Subscriber scrubbedScanSub;

 	
public:
  AssertiveBehaviour(ros::NodeHandle& nh);
  ~AssertiveBehaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class AssertiveBehaviour

#endif // ASSERTIVE_BEHAVIOUR_H
