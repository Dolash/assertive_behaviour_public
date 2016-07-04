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

        startX = 1.53;
        startY = -2.9;

        /**/
        goalX = -1.47;
        goalY = -2.9;
        
        startupLights = 0;
        fightStartLights = 1;
        loseFightLights = 2;
        winFightLights = 3;
        backToNormalLights = 4;
        
        subjectDetected = false;
        
        brave = false;
        defeat = false;
        timer = ros::Time::now();
        aggression = 6;
  
  
        startupSound = 18;
        fightStartSound = 0;
        loseFightSound = 0;
        winFightSound = 0;
        backToNormalSound = 0;

  	/*Subscriber for the laser-based leg detector*/
  	legSub = nh.subscribe("/legs", 1, &AssertiveBehaviour::legCallback, this);
  	/*If we're using the vicon, subscribe for our pose that way*/
  	if (viconMode == true)
  	{
  	    poseSub = nh.subscribe("/global_poses", 1, &AssertiveBehaviour::viconCallback, this);
  	    subjectPoseSub = nh.subscribe("/vicon/subject/subject", 1, &AssertiveBehaviour::viconSubjectCallback, this);
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
    keyframe_pub = nh.advertise<autonomy_leds_msgs::Keyframe>("/leds/display", 1);
    
    audio_cmd.data = startupSound;
    audio_pub.publish(audio_cmd);
    setLights(startupLights);
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
void AssertiveBehaviour::viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
    latestPoses = *pose;
    poseReceived = true;
}

/*For the vicon subscriber for the subject, extracting the data from the message.*/
void AssertiveBehaviour::viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
    latestSubjectPoses = *pose;
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

void AssertiveBehaviour::setLights(int setting)
{
    if (setting == startupLights)
    {
        lights.duration = 10;
        lights.start_index = 1;
        lights.color_pattern[1].r = 0;
        lights.color_pattern[1].g = 0;
        lights.color_pattern[1].b = 1;
        
    }
    else if (setting == startupLights)
    {
    
    }
    else if (setting == fightStartLights)
    {
    
    }
    else if (setting == winFightLights)
    {
    
    }
    else if (setting == loseFightLights)
    {
    
    }
    keyframe_pub.publish(lights);
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
	float yaw = tf::getYaw(latestPoses.transform.rotation);
	/*The obstacle avoider checks if the desired angle is within front 180* arc and if so uses laser for obstacle avoiding. This could obviously cause problems if there's no open path forward*/
	if (((yaw > (result - 0.785)) && (yaw < (result + 0.785))) || (((yaw > 2.36) && (result < -2.36)) || ((yaw < -2.36) && (result > 2.36))))
	{
	    result = obstacleAvoider(result);
	}
	
	return result;
}

float AssertiveBehaviour::obstacleAvoider(float desired)
{

    return desired;

      int laserStartIndex = 0;
      int laserWindowRadius = 10;
      bool clear = true;
      bool triedRight = false;
      float possible = desired;
      //float yaw = tf::getYaw(latestPoses.transform.rotation);
      
    

		//float difference = yaw - desired;
		laserStartIndex = (desired / 1.57595) * 180;
		
      
      
      while (panicking == false)
      {
          for (int i = (laserStartIndex - laserWindowRadius); i < (laserStartIndex + laserWindowRadius); i++)
          {
            if (latestLaserScan[i] < 0.15)
            {
                clear = false;
            }
          }
          /*You found an open way forward!*/
          if (clear == true)
          {
              return possible;
          }
          /*If this laser window was not clear, try shifting your angle and search again*/
          else
          {
            if (triedRight == false)
            {
                laserStartIndex += 5;
                possible = ((laserStartIndex/180) * 1.57595);
            }
            else
            {
                laserStartIndex -= 5;
                possible = ((laserStartIndex/180) * 1.57595); 
            }
          }
          /*You found no opening to the right of your desired angle, so try the left.*/
          if (triedRight == false && laserStartIndex + laserWindowRadius > 180)
          {
            triedRight = true;
            
          }
          /*You've searched everywhere and found no opening! Panic!*/
          else if (triedRight == true && laserStartIndex - laserWindowRadius < 0)
          {
            panicking = true;
          }
      }
      /*If you made it here, you found no way forward, so the panic variable was set and you'll be doing that instead.*/
      return 0;
}


/*By using your pose and the goal location pose, this function will determine the velocity command it should next issue to continue a normal journey.*/
/*TODO: This is using quaternions since I'm assuming vicon, might need to adjust.*/
void AssertiveBehaviour::waypointing()
{
        float desiredX;
        float desiredY;
        float desiredAngle = 0;
        /*Since the vicon bridge is giving us quaternions, we'll want to get our own yaw back out of it.*/
        //float yaw = tf::getYaw(latestPoses.poses[0].orientation);
        float yaw = tf::getYaw(latestPoses.transform.rotation);
        
        
        
        if (returnTrip == false)
        {
            desiredX = goalX;
            desiredY = goalY;
        }
        else
        {
            desiredX = startX;
            desiredY = startY;
        }
        /*Check if you've arrived at your destination and should switch goals*/
        if (fabs(latestPoses.transform.translation.x - desiredX) < 0.2 && fabs(latestPoses.transform.translation.y - desiredY) < 0.2)
        {
            returnTrip = !returnTrip;
        }

	/*Now we calculate the yaw we'd want from our current position to be driving toward the goal.*/
		desiredAngle = getDesiredAngle(desiredX, desiredY, latestPoses.transform.translation.x, latestPoses.transform.translation.y);
		/*If we're moving and our current yaw is within 0.25 of what we want, keep driving*/
		if ((yaw > (desiredAngle - 0.25)) && (yaw < (desiredAngle + 0.25)) && (driving == true))
		/*if ((yaw > (desiredAngle - 0.25)) && (yaw < (desiredAngle + 0.25)) && (driving == true))*/

		{
			move_cmd.linear.x = 0.5;
			move_cmd.angular.z = 0.0;
		}
		/*However, if we're near the "seam" where ~3.1415 becomes ~-3.1415, we need to be fiddly if our desired angle is on the other side*/
		else if ((((yaw > 2.9) && (desiredAngle < -2.9)) || ((yaw < -2.9) && (desiredAngle > 2.9))) && (driving == true))
		{
			move_cmd.linear.x = 0.5;
			move_cmd.angular.z = 0.0;
		}
		/*Now, if we're currently turning and have successfully turned to within 0.15 of what we want, start driving again.*/
		else if ((yaw > (desiredAngle - 0.15)) && (yaw < (desiredAngle + 0.15)) && (driving == false))

		{
			move_cmd.linear.x = 0.5;
			move_cmd.angular.z = 0.0;
			driving = true;
		}
		/*And again, if we're within ~0.15 but it's at one of the borders, allow it too.*/
		else if ((((yaw > 3) && (desiredAngle < -3)) || ((yaw < -3) && (desiredAngle > 3))) && (driving == false))
		{
			move_cmd.linear.x = 0.5;
			move_cmd.angular.z = 0.0;
			driving = true;
		}
		/*If we get here then the difference between our yaw and desired angle is too great, moving or not, seam or no seam, so start turning*/
		else
		{
			move_cmd.linear.x = 0.0;
			move_cmd.angular.z = 0.5;
			driving = false;
		}
		cmd_vel_pub.publish(move_cmd);
	//cmd_vel_pub.publish(move_cmd);
    ROS_INFO("[ASSERTIVE_BEHAVIOUR] In waypointing");

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
/*Deprecated since I want to build a larger sensor-mediated function.*/
/*void AssertiveBehaviour::legAhead()
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
}*/

/*When this behaviour is being used with the vicon, this function detects if there is a human or robot subject in the robot's way using their vicon poses. This is an alternative to a sensor-mediated version.*/
/*TODO: Even if I'm not using vicon data to control, I may want to be logging it all.*/
void AssertiveBehaviour::viconSubjectAhead()
{
    float angleToSubject = getDesiredAngle(latestSubjectPoses.transform.translation.x, latestSubjectPoses.transform.translation.y, latestPoses.transform.translation.x, latestPoses.transform.translation.y);
    float yaw = tf::getYaw(latestPoses.transform.rotation);
    
    /*Take your pose and orientation compare to the pose of every other tracked subject and if one is detected then set that detection to true*/
    if (fabs(latestPoses.transform.translation.x - latestSubjectPoses.transform.translation.x) < 0.5 
    && fabs(latestPoses.transform.translation.y - latestSubjectPoses.transform.translation.y) < 0.5 
    && ((yaw > (angleToSubject - 0.25)) && (yaw < (angleToSubject + 0.25)) || (((yaw > 2.9) && (angleToSubject < -2.9)) 
    || ((yaw < -2.9) && (angleToSubject > 2.9)))))
    {
        subjectDetected = true;
    }
    else 
    {
        subjectDetected = false;
    }

}

/*This function detects if there is a human or robot subject in the robot's way using all available sensors and detectors, or if vicon mode is enabled will call that version of this function instead.*/
void AssertiveBehaviour::subjectAhead()
{
    if (viconMode == true)
    {
        viconSubjectAhead();
    }
    else
    {
        /*TODO: Sensor-mediated detection using autonomy hri package modules*/
        
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

    /*TODO: Set good cmd_vel values when retreating*/
    /*First check if you've lost the fight and if so get out of the way*/
    if (defeat == true)
    {
        reverseClearance();
        if (behindRightClear == true)
        {
            move_cmd.linear.x = 0.5;
            move_cmd.angular.z = -0.5;
        }
        else if (behindLeftClear == true)
        {
            move_cmd.linear.x = -0.5;
            move_cmd.angular.z = -0.5;
        }
        else if (behindMiddleClear == true)
        {
            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = 0.0;
        }
        else 
        {
            /*You're helpless! Hopefully they get out of your way?*/
            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = -0.5;
        }
       cmd_vel_pub.publish(move_cmd);
    }

    else /*Fight ongoing*/
    {
        float distInitial;
        float distCurrent;
        if (viconMode == true)
        {
            /*How far were you from the subject at the start of the fight?*/
            distInitial = sqrt(pow((latestPoses.transform.translation.x - initialX), 2) + pow((latestPoses.transform.translation.y - initialY), 2));
            /*How far are you now?*/
            distCurrent = sqrt(pow((latestPoses.transform.translation.x - latestSubjectPoses.transform.translation.x), 2) + pow((latestPoses.transform.translation.y - latestSubjectPoses.transform.translation.y), 2));
        }
        else
        {
            /*TODO: Get the relevant distances from sensor detection*/
            
        }
        
        if (distInitial > distCurrent + 0.25) /*If they have moved toward you, you lose*/
        {
            audio_cmd.data = loseFightSound;
            audio_pub.publish(audio_cmd);
            setLights(loseFightLights);
            defeat = true;
        }
        else if(distCurrent > distInitial + 0.25) /*If they have moved away from you, you win!*/
        {
            audio_cmd.data = winFightSound;
            audio_pub.publish(audio_cmd);
            setLights(winFightLights);
            brave = true;
            fighting = false;
            navigating = true;
        }
        else if (ros::Time::now() - timer >  ros::Duration(aggression)) /*Your patience ran out, so you win! Probably!*/
        {
            audio_cmd.data = winFightSound;
            audio_pub.publish(audio_cmd);
            setLights(winFightLights);
            brave = true;
            fighting = false;
            navigating = true;
        }
        else /*keep waiting*/
        {
            /*Maybe should gradient to red lights, increase hum to signal patience running out?*/
        }
        
    }    

}


/*This behaviour is called if something is within emergency range or some other trapped/deadlocked scenario. One consideration is making the difference between "too close" as mindless obstacle and "too close" as other robot/person, where expressing displeasure at being pushed may be warranted.*/
void AssertiveBehaviour::panickingBehaviour()
{
/*TODO: Fill this in. Stop? Spin? Back away?*/

    move_cmd.linear.x = 0.0;
    move_cmd.angular.z = 0.0;
    cmd_vel_pub.publish(move_cmd);
}


/*This is the normal driving behaviour when either no subject is ahead or else you won the fight. Handles driving as well as obstace and subject detection.*/
void AssertiveBehaviour::navigatingBehaviour()
{

    if ((viconMode == true && poseReceived == true) || (viconMode == false))
    {
        viconSubjectAhead();
        /*If someone is newly detected ahead of you, a fight starts*/
        if (subjectDetected == true  && brave == false)
        {
            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = 0.0;
            cmd_vel_pub.publish(move_cmd);
            navigating == false;
            fighting == true;
            timer = ros::Time::now();
            initialX = latestSubjectPoses.transform.translation.x;
            initialY = latestSubjectPoses.transform.translation.y;
            audio_cmd.data = fightStartSound;
            audio_pub.publish(audio_cmd);
            setLights(fightStartLights);
        }
        else if (subjectDetected == true  && brave == true) /*Someone is in front of you but you won a fight so keep pushing*/
        {
                timer = ros::Time::now();
                waypointing();
        }
        else if (subjectDetected == false  && brave == true) /*you won a fight but there's no longer someone in front of you*/
        {
            if (timer + ros::Duration(3) < ros::Time::now()) /*If they haven't been re-detected in front of you in 3 seconds stop being brave*/
            {
                audio_cmd.data = backToNormalSound;
                audio_pub.publish(audio_cmd);
                setLights(backToNormalLights);
                brave = false;
            }
            waypointing();
        } /*Normal driving*/
        else
        {
            waypointing();
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
