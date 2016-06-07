#include <tf/transform_datatypes.h>
#include "assertive_behaviour/assertive_behaviour.h"

/*Jacob's function for extracting the yaw from the Quaternion you get from Vicon. Thanks, Jacob!*/
/*double getYaw(const geometry_msgs::Quaternion& quat) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  return y;
}*/

AssertiveBehaviour::AssertiveBehaviour(ros::NodeHandle& nh) 
    : nh(nh),
    privNh("~") {

        privNh.param<double>("loop_hz", loopHz, 30);
        
        cycleStartTime = ros::Time::now();
        panicking = false;
        fighting = false;
        navigating = true;

        legReceived = false;
        sonarReceived = false;
        legWarning = false;
        poseReceived = false;
        returnTrip = false;
        driving = false;
        laserReceived = false;
        behindRightClear = false;
        behindLeftClear = false;
        behindMiddleClear = false;
        viconMode = true;


  

  	/*Subscriber for the laser-based leg detector*/
  	legSub = nh.subscribe("/legs", 1, &AssertiveBehaviour::legCallback, this);
  	/*If we're using the vicon, subscribe for our pose that way*/
  	if (viconMode == true)
  	{
  	    poseSub = nh.subscribe("/global_poses", 1, &AssertiveBehaviour::viconCallback, this);
  	}
  	else
  	{
  	    /*However else we're going to get our pose in non-vicon situations*/
  	}
  	laserSub = nh.subscribe("/scan", 1, &AssertiveBehaviour::laserCallback, this);
  	sonarSub = nh.subscribe("/sonar", 1, &AssertiveBehaviour::sonarCallback, this);
	
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
	gripper_pub = nh.advertise<p2os_driver::GripperState>("/gripper_control", 30);
	audio_pub = nh.advertise<std_msgs::UInt16>("/sound_player", 1);
	led_pub = nh.advertise<autonomy_leds_msgs::LED>("/leds/set_led", 1);

    audio_cmd.data = 18;
    audio_pub.publish(audio_cmd);
	ROS_INFO("[ASSERTIVE_BEHAVIOUR] Initialized.");
}

AssertiveBehaviour::~AssertiveBehaviour() {
 	ROS_INFO("[ASSERTIVE_BEHAVIOUR] Destroyed.");
}

/*For the leg subscriber, extracting the data from the message.*/
void AssertiveBehaviour::legCallback(const geometry_msgs::PoseArray legData) {
    latestLegPoseArray = legData;
    legReceived = true;
}

/*For the vicon subscriber, extracting the data from the message.*/
void AssertiveBehaviour::viconCallback(const geometry_msgs::PoseArray poseData) {
    latestPoses = poseData;
    poseReceived = true;
}

/*For the laser subscriber, extracting the data from the message.*/
void AssertiveBehaviour::laserCallback(const sensor_msgs::LaserScan scanData) {
    latestLaserScan = scanData.ranges;
    laserReceived = true;
}

/*For the sonar subscriber, extracting the data from the message.*/
//void AssertiveBehaviour::sonarCallback(const p2os_driver::SonarArray::ConstPtr & sonarData) {
void AssertiveBehaviour::sonarCallback(const p2os_driver::SonarArray sonarData) {
    latestSonarScan = sonarData.ranges;
    sonarReceived = true;
}


/*For calculating the angle we want to be turned in order to be facing our target, given your pose and the goal location's pose*/
float AssertiveBehaviour::getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn)
{
	float result = 0;

	//I'm making this adjustment because I am turning the robot's current position into the origin, so the origin x and y are really currentPositionX/Y - currentPositionX/Y.
	//I'm doing this for my brain's sake.
	float nTargetX = (targetX - currentXCoordinateIn);
	float nTargetY = (targetY - currentYCoordinateIn);


	/*So this calculates, if our robot was at the origin and our target is at the appropriate relative position, what angle should we be turned to face them?*/
	float angbc = atan2((nTargetY), (nTargetX));
	result = angbc;
	result = (result - 1.57595);
	/*A quick fix in the event that this desired angle adjustment takes us "off the edge" of pi*/
	if (result < -3.1518)
    {
		result = (3.1518 - (fabs(result) - 3.1518));
	}
	return result;
}


/*By using your pose and the goal location pose, this function will determine the velocity command it should next issue to continue a normal journey.*/
/*TODO: This is using quaternions since I'm assuming vicon, might need to adjust.*/
void AssertiveBehaviour::waypointing()
{
        float desiredX;
        float desiredY;
        float desiredAngle = 0;
        /*Since the vicon bridge is giving us quaternions, we'll want to get our own yaw back out of it.*/
        float yaw = tf::getYaw(latestPoses.poses[0].orientation);
        if (returnTrip == false)
        {
            desiredX = 1;
            desiredY = 1;
        }
        else
        {
            desiredX = 0;
            desiredY = 1;
        }

	/*Now we calculate the yaw we'd want from our current position to be driving toward the goal.*/
		desiredAngle = getDesiredAngle(desiredX, desiredY, latestPoses.poses[0].position.x, latestPoses.poses[0].position.y);
		/*If we're moving and our current yaw is within 0.25 of what we want, keep driving*/
		if ((yaw > (desiredAngle - 0.25)) && (yaw < (desiredAngle + 0.25)) && (driving == true))
		/*if ((yaw > (desiredAngle - 0.25)) && (yaw < (desiredAngle + 0.25)) && (driving == true))*/

		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
		}
		/*However, if we're near the "seam" where ~3.1415 becomes ~-3.1415, we need to be fiddly if our desired angle is on the other side*/
		else if ((((yaw > 2.9) && (desiredAngle < -2.9)) || ((yaw < -2.9) && (desiredAngle > 2.9))) && (driving == true))
		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
		}
		/*Now, if we're currently turning and have successfully turned to within 0.15 of what we want, start driving again.*/
		else if ((yaw > (desiredAngle - 0.15)) && (yaw < (yaw + 0.15)) && (driving == false))

		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
			driving = true;
		}
		/*And again, if we're within ~0.15 but it's at one of the borders, allow it too.*/
		else if ((((yaw > 3) && (desiredAngle < -3)) || ((yaw < -3) && (yaw > 3))) && (driving == false))
		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
			driving = true;
		}
		/*If we get here then the difference between our yaw and desired angle is too great, moving or not, seam or no seam, so start turning*/
		else
		{
			move_cmd.linear.x = 0.0;
			move_cmd.angular.z = 0.2;
			driving = false;
		}
	cmd_vel_pub.publish(move_cmd);

}

/*The functions that open and close the gripper.*/
void AssertiveBehaviour::openGripper()
{
    /*TODO: Check message format for this*/
    gripper_pub.publish(grip_cmd);
}
void AssertiveBehaviour::closeGripper()
{
    /*TODO: Check message format for this*/
    gripper_pub.publish(grip_cmd);
}


/*This function uses the leg detector to detect humans stopped in the front arc.*/
/*TODO: Fill out or integrate into a proper human detector/subject detector*/
void AssertiveBehaviour::legAhead()
{
    if (legReceived == true)
    {
        legWarning = false;
        for (int i = 0; i < latestLegPoseArray.poses.size(); i++)
        {
            if (std::fabs(latestLegPoseArray.poses[i].position.x) < 0.4 && std::fabs(latestLegPoseArray.poses[i].position.y) < 0.4)
            {
                legWarning = true;
                ROS_INFO("[ASSERTIVE_BEHAVIOUR] Forward leg reading: %f, %f.", latestLegPoseArray.poses[i].position.x, latestLegPoseArray.poses[i].position.y );
            }
        }
        ROS_INFO("[ASSERTIVE_BEHAVIOUR] Forward leg reading: %d.", legWarning);
    }
    else
    {
        ROS_INFO("[ASSERTIVE_BEHAVIOUR] No leg data received");
    }   
}

/*When this behaviour is being used with the vicon, this function detects if there is a human or robot subject in the robot's way using their vicon poses. This is an alternative to a sensor-mediated version.*/
/*TODO: Even if I'm not using vicon data to control, I may want to be logging it all.*/
void AssertiveBehaviour::viconSubjectAhead()
{

    /*Take your pose and orientation compare to the pose of every other tracked subject and if one is detected then set that detection to true*/

}


/*A simple function for testing the clearance behind the robot, to be called before backing up
TODO: Replace this with sliding box*/
void AssertiveBehaviour::reverseClearance()
{
    if (sonarReceived == true)
    {
        if(latestSonarScan[8] > 0.5 && latestSonarScan[9] > 0.5 && latestSonarScan[10] > 0.5)
        {
            behindRightClear = true;
        }
        else
        {
            behindRightClear = false;
        }
        if(latestSonarScan[13] > 0.5 && latestSonarScan[14] > 0.5 && latestSonarScan[15] > 0.5)
        {
            behindLeftClear = true;
        }
        else
        {
            behindLeftClear = false;
        }
        if(latestSonarScan[11] > 0.5 && latestSonarScan[12] > 0.5)
        {
            behindMiddleClear = true;
        }
        else
        {
            behindMiddleClear = false;
        }
    }
    else
    {
    
    }
}


/*When a fight is initiated, the robot remains in this state until its forward interlocutor is cleared. The robot will either be unsure, or else if it decides the time is right it will advance, or else if it is being advanced upon it will back up.*/
void AssertiveBehaviour::fightingBehaviour()
{

    /*Advancing*/
    /*TODO: Write this whole thing*/
    
    /*Backing up*/
    /*TODO: Replace this with sliding box*/
    reverseClearance();
    if (behindRightClear == true)
    {
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
    }
    else if (behindLeftClear == true)
    {
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
    }
    else if (behindMiddleClear == true)
    {
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
    }
    else 
    {
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
    }
    cmd_vel_pub.publish(move_cmd);
}


/*This behaviour is called if something is within emergency range or some other trapped/deadlocked scenario. One consideration is making the difference between "too close" as mindless obstacle and "too close" as other robot/person, where expressing displeasure at being pushed may be warranted.*/
void AssertiveBehaviour::panickingBehaviour()
{
/*TODO: Fill this in. Stop? Spin? Back away?*/

    move_cmd.linear.x = 0.0;
    move_cmd.angular.z = 0.0;
    cmd_vel_pub.publish(move_cmd);
}


/*This is the normal driving behaviour when no interlocutor is detected and there's no obstacle-avoiding issues.*/
void AssertiveBehaviour::navigatingBehaviour()
{
/*TODO: Fill out. Use pose information or a map to decide where to go with a goal in mind, maintain distance, and be ready to switch states if someone gets in your way or something gets too close.*/
    if (poseReceived == true)
    {
        //legAhead();
        if (legWarning == false)
        { 
            waypointing();
        }
        else
        {
            ROS_INFO("[ASSERTIVE_BEHAVIOUR] Warning: Leg, stopping");
            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = 0.0;
            cmd_vel_pub.publish(move_cmd);
        }
    }
    else
    {
        ROS_INFO("[ASSERTIVE_BEHAVIOUR] No Vicon pose received.");
    }
}

/*One run of the loop, picks which behaviour to do depending on what state the robot is in. The different behaviours control state transitions based on their own criteria, this is just where the program reassesses which one to apply.*/
void AssertiveBehaviour::spinOnce() {

    if(panicking)
    {
        panickingBehaviour();
    }
    else if(fighting)
    {
        fightingBehaviour();

    }
    else if(navigating)
    {
        navigatingBehaviour();
    }
    else
    {
        ROS_INFO("[ASSERTIVE_BEHAVIOUR] Stateless");
    }
	
  	ros::spinOnce();
}

/*Gets called by main, runs all the time at the given rate.*/
void AssertiveBehaviour::spin() {
  ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
  }
}
