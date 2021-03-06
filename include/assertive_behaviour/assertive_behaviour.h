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
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <p2os_msgs/GripperState.h>
#include <std_msgs/Float32.h>
#include <p2os_msgs/SonarArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>
#include <autonomy_leds_msgs/Keyframe.h>
#include <vector>
#include <unistd.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <rosgraph_msgs/Clock.h>

#define PI 3.14159
#define TWO_PI 6.283185*/

class AssertiveBehaviour {
private:

	ros::Time timer;
	ros::Time moveOrderTimer;
	ros::Time beginTimer;
	rosgraph_msgs::Clock clockTime;
    bool panicking;
    bool fighting;
    bool navigating;
    bool driving;

	bool unpaused;
	bool listeningForUnpause;


	sensor_msgs::LaserScan scrubbedScan;
    /*This stores the latest laser scan's 180 ranges*/
	std::vector<float> latestLaserScan;
	/*This stores the latest sonar scan's 16 ranges*/
	std::vector<double> latestSonarScan;
	/*These bools confirm that the laser and sonar are transmitting, so that they won't be used unless these are triggered.*/
    bool laserReceived;
    bool sonarReceived;
	bool amclReceived;
    geometry_msgs::TransformStamped latestPoses;
    geometry_msgs::TransformStamped latestSubjectPoses;


	std::string poseTopic;
	std::string scrubbedLaserTopic;
	std::string cmdVelTopic;
	std::string scanTopic;
	std::string amclTopic;
	std::string joyTopic;

    
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
	bool doorReached;
 	
    float startX;
    float startY;
	
    float goalX;
    float goalY;

	float startYaw;
	float goalYaw;

    float startDoorX;
    float startDoorY;
	
    float goalDoorX;
    float goalDoorY;

	float startDoorYaw;
	float goalDoorYaw;

	float loseDistance;
	float winDistance;

        int toleranceThreshold;
    bool subjectDetected;
    
	float distInitial;
	float angleAtDefeat;
	bool unwinding;
	std_msgs::Bool emergencyPause;
	bool fightSoundPlayed;

    float initialX;
    float initialY;
    
    bool defeat;	
    float aggression;

	nav_msgs::Odometry stagePose;

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
    
	geometry_msgs::PoseWithCovarianceStamped amclPose;



    bool fightStarting;
	bool firstGoal;
	bool stageMode;

	bool humanReceived;

	bool scrubbedScanReceived;
	bool clockReceived;

  	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
  	void stagePoseCallback(const nav_msgs::Odometry::ConstPtr& pose);
  	void viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
  	void laserCallback(const sensor_msgs::LaserScan scanData);
  	void sonarCallback(const p2os_msgs::SonarArray sonarData);
	void cmdVelListenerCallback(const geometry_msgs::Twist navData);
	void localMaximaCallback(const geometry_msgs::PoseArray maximaData);
	void scrubbedScanCallback(const sensor_msgs::LaserScan scanData);
	void clockCallback(const rosgraph_msgs::Clock clockData);
	void amclCallback(const geometry_msgs::PoseWithCovarianceStamped amclData);
	void emergencyStopCallback(const std_msgs::Bool emergencyPauseData);
	void joyCallback(const sensor_msgs::Joy joyMessage);

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
	/*Publisher to set the movement goal*/
	ros::Publisher winner_pub;


	/*Movement orders for Pioneer*/
	geometry_msgs::Twist move_cmd;
    /*An audio request*/
    std_msgs::UInt16 audio_cmd;
    /*A gripper command*/
    p2os_msgs::GripperState grip_cmd;
	/*A single keyframe which can be used to set the leds different colours. Kept simple instead of animated for now.*/
    autonomy_leds_msgs::Keyframe lights;

	std_msgs::Bool winner;
	
 
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

	ros::Subscriber clockSub;

	ros::Subscriber amclSub;

	ros::Subscriber emergencyStopSub;

	ros::Subscriber joySub;

 	
public:
  AssertiveBehaviour(ros::NodeHandle& nh);
  ~AssertiveBehaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class AssertiveBehaviour

#endif // ASSERTIVE_BEHAVIOUR_H
