#include <tf/transform_datatypes.h>
#include "assertive_behaviour/assertive_behaviour.h"

AssertiveBehaviour::AssertiveBehaviour(ros::NodeHandle& nh) 
    : nh(nh),
    privNh("~") {

        privNh.param<double>("loop_hz", loopHz, 10);
        
privNh.param<bool>("vicon_mode", viconMode, false);
privNh.param<bool>("stage_mode", stageMode, true);

	privNh.param<std::string>("pose_topic", poseTopic, "/pose");
	privNh.param<std::string>("scrubbed_scan_topic", scrubbedLaserTopic, "/scrubbed_scan");
	privNh.param<std::string>("cmd_vel_topic", cmdVelTopic, "/cmd_vel");
	privNh.param<std::string>("scan_topic", scanTopic, "/scan");
	privNh.param<std::string>("amcl_topic", amclTopic, "/amcl_pose");
	privNh.param<std::string>("joy_topic", joyTopic, "/joy_pose");

	privNh.param<float>("start_x", startX, 1);
privNh.param<float>("start_y", startY, 1);
privNh.param<float>("end_x", goalX, 1);
privNh.param<float>("end_y", goalY, 2);
privNh.param<float>("start_yaw", startYaw, 0);
privNh.param<float>("end_yaw", goalYaw, 0);
privNh.param<float>("start_door_x", startDoorX, 1);
privNh.param<float>("start_door_y", startDoorY, 1);
privNh.param<float>("end_door_x", goalDoorX, 1);
privNh.param<float>("end_door_y", goalDoorY, 2);
privNh.param<float>("start_door_yaw", startDoorYaw, 0);
privNh.param<float>("end_door_yaw", goalDoorYaw, 0);

privNh.param<int>("detection_tolerance", toleranceThreshold, 4);
privNh.param<float>("aggression", aggression, 8);
privNh.param<float>("lose_distance", loseDistance, 0.5);
privNh.param<float>("win_distance", winDistance, 0.5);
        panicking = false;
	listeningForUnpause = false;
	unpaused = false;
        fighting = false;
        navigating = true;
        sonarReceived = false;
        poseReceived = false;
        returnTrip = false;
        driving = false;
        laserReceived = false;
        behindRightClear = false;
        behindLeftClear = false;
        behindMiddleClear = false;
	humanReceived = false;
        fightStarting = false;
        firstTime = false;
	clockReceived = false;
	amclReceived = false;
      	firstGoal = false;
	unwinding = false;
	emergencyPause.data = false;
	doorReached = true;
        startupLights = 0;
        fightStartLights = 1;
        loseFightLights = 2;
        winFightLights = 3;
        backToNormalLights = 4;
        fightSoundPlayed = false;
	winner.data = false;

        
        temp.r = 0;
        temp.g = 0;
        temp.b = 0;
        temp.a = 0;
        for (int i = 0; i < 1; i++)
        {
            
            lightColours.push_back(temp);
        }
        lights.color_pattern = lightColours; 
        
        subjectDetected = false;
        
        brave = false;
        defeat = false;
	if(stageMode == false)
	{
        	timer = ros::Time::now();
		beginTimer = ros::Time::now();
		moveOrderTimer = ros::Time::now() - ros::Duration(30);
	}        
	//aggression = 9;
  
        obstacle = false;
	
	scrubbedScanReceived = false;
  
        startupSound = 22;
        fightStartSound = 23;
        loseFightSound = 24;
        winFightSound = 25;
        backToNormalSound = 26;


  	/*If we're using the vicon, subscribe for our pose that way*/
  	if (viconMode == true)
  	{
  	    poseSub = nh.subscribe("/global_poses", 1, &AssertiveBehaviour::viconCallback, this);
  	    subjectPoseSub = nh.subscribe("/vicon/subject/subject", 1, &AssertiveBehaviour::viconSubjectCallback, this);
  	}
	else if (stageMode == true)
	{
		poseSub = nh.subscribe(poseTopic, 1, &AssertiveBehaviour::stagePoseCallback, this);
		scrubbedScanSub = nh.subscribe(scrubbedLaserTopic, 1, &AssertiveBehaviour::scrubbedScanCallback, this);
	}
  	else
  	{
		goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
		cmdVelListenerSub = nh.subscribe("/listener/cmd_vel", 1, &AssertiveBehaviour::cmdVelListenerCallback, this);
	//	humanMaximaSub = nh.subscribe("/human/local_maxima", 1, &AssertiveBehaviour::localMaximaCallback, this);
		scrubbedScanSub = nh.subscribe(scrubbedLaserTopic, 1, &AssertiveBehaviour::scrubbedScanCallback, this);
		amclSub = nh.subscribe(amclTopic, 1, &AssertiveBehaviour::amclCallback, this);
  	}
  	laserSub = nh.subscribe(scanTopic, 1, &AssertiveBehaviour::laserCallback, this);
  	sonarSub = nh.subscribe("/sonar", 1, &AssertiveBehaviour::sonarCallback, this);
	emergencyStopSub = nh.subscribe("/emergency_stop", 1, &AssertiveBehaviour::emergencyStopCallback, this);
	joySub = nh.subscribe(joyTopic, 1, &AssertiveBehaviour::joyCallback, this);

	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 30);
	gripper_pub = nh.advertise<p2os_msgs::GripperState>("/gripper_control", 30);
	audio_pub = nh.advertise<std_msgs::UInt16>("/sound_player", 1, true);
    	keyframe_pub = nh.advertise<autonomy_leds_msgs::Keyframe>("/leds/display", 1);
	winner_pub = nh.advertise<std_msgs::Bool>("/winner", 1);
	
    	

    clockSub = nh.subscribe("/clock", 1, &AssertiveBehaviour::clockCallback, this);
	ROS_INFO("[ASSERTIVE_BEHAVIOUR] Initialized.");
}

AssertiveBehaviour::~AssertiveBehaviour() {
 	ROS_INFO("[ASSERTIVE_BEHAVIOUR] Destroyed.");
}

/*For the vicon subscriber, extracting the data from the message.*/
void AssertiveBehaviour::viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
    latestPoses = *pose;
    poseReceived = true;
}

void AssertiveBehaviour::stagePoseCallback(const nav_msgs::Odometry::ConstPtr& pose) {
    stagePose = *pose;
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] stage pose.");
    poseReceived = true;
}

/*For the vicon subscriber for the subject, extracting the data from the message.*/
void AssertiveBehaviour::viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {

    latestSubjectPoses = *pose;
}

void AssertiveBehaviour::joyCallback(const sensor_msgs::Joy joyMessage) {
	ROS_INFO("[ASSERTIVE_BEHAVIOUR] Joy message received.");
	if (listeningForUnpause == true)
	{
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] listening.");
		if (joyMessage.buttons[1] == 1)
		{
			unpaused = true;
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Unpaused.");
		}
		else
		{
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Wrong button pressed or no button pressed.");
		}
	}
	else
	{
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] Not listening.");
	}
}

/*For the laser subscriber, extracting the data from the message.*/
void AssertiveBehaviour::laserCallback(const sensor_msgs::LaserScan scanData) {
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] laser scan.");
    latestLaserScan = scanData.ranges;
    laserReceived = true;
}

/*For the sonar subscriber, extracting the data from the message.*/
//void AssertiveBehaviour::sonarCallback(const p2os_driver::SonarArray::ConstPtr & sonarData) {
void AssertiveBehaviour::sonarCallback(const p2os_msgs::SonarArray sonarData) {
    latestSonarScan = sonarData.ranges;
   sonarReceived = true;
}

void AssertiveBehaviour::cmdVelListenerCallback(const geometry_msgs::Twist navData)
{
	if (navigating == true && emergencyPause.data == false)
	{
		cmd_vel_pub.publish(navData);
	}
}

void AssertiveBehaviour::amclCallback(const geometry_msgs::PoseWithCovarianceStamped amclData) {
   amclPose = amclData;
   amclReceived = true;
}

void AssertiveBehaviour::localMaximaCallback(const  geometry_msgs::PoseArray maximaData) {
   humanMaximaArray = maximaData;
   humanReceived = true;
}


void AssertiveBehaviour::scrubbedScanCallback(const sensor_msgs::LaserScan scanData)
{
	scrubbedScan = scanData;
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] scrubbed scan.");
	scrubbedScanReceived = true;
}


void AssertiveBehaviour::clockCallback(const rosgraph_msgs::Clock clockData)
{
	clockTime = clockData;
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Clock.");
	clockReceived = true;
}

void AssertiveBehaviour::emergencyStopCallback(const std_msgs::Bool emergencyPauseData)
{
	emergencyPause = emergencyPauseData;
	
}

void AssertiveBehaviour::setLights(int setting)
{

    /*if (setting == 0)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 1;
            lights.color_pattern[i].g = 1;
            lights.color_pattern[i].b = 1;
            lights.color_pattern[i].a = 0;
        }
        
    }
    else if (setting == 1)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 0;
            lights.color_pattern[i].g = 1;
            lights.color_pattern[i].b = 1;
            lights.color_pattern[i].a = 0;
        }
    }
    else if (setting == 2)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 1.0;
            lights.color_pattern[i].g = 0;
            lights.color_pattern[i].b = 0;
            lights.color_pattern[i].a = 0;
        }
    }
    else if (setting == 3)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 0;
            lights.color_pattern[i].g = 1;
            lights.color_pattern[i].b = 0;
            lights.color_pattern[i].a = 0;
        }
    }
    else if (setting == 4)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
		if (returnTrip == true)
		{
            		lights.color_pattern[i].r = 0;
            		lights.color_pattern[i].g = 0;
            		lights.color_pattern[i].b = 1;
            		lights.color_pattern[i].a = 0;
		}
		else
		{
			lights.color_pattern[i].r = 1;
           	 	lights.color_pattern[i].g = 1;
            		lights.color_pattern[i].b = 0;
            		lights.color_pattern[i].a = 0;
		}
        }
    }*/
	lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
		if (aggression < 5)
		{
            		lights.color_pattern[i].r = 1;
            		lights.color_pattern[i].g = 0;
            		lights.color_pattern[i].b = 0;
            		lights.color_pattern[i].a = 0;
		}
		else
		{
			lights.color_pattern[i].r = 0;
            		lights.color_pattern[i].g = 1;
            		lights.color_pattern[i].b = 0;
            		lights.color_pattern[i].a = 0;
		}
        }

    keyframe_pub.publish(lights);
}


/*For calculating the angle we want to be turned in order to be facing our target, given your pose and the goal location's pose*/
float AssertiveBehaviour::getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn, bool destination)
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
	if (result < -3.1415)
    {
		//result = (3.1518 - (fabs(result) - 3.1518));
		result += 6.2831;
	}
	else if (result > 3.1415)
	{
	    result -= 6.2831;
	}
	//float yaw = tf::getYaw(latestPoses.transform.rotation);
	/*The obstacle avoider checks if the desired angle is within front 180* arc and if so uses laser for obstacle avoiding. This could obviously cause problems if there's no open path forward*/
	
	
	//float adjustedDesired = result - yaw;
        //if (adjustedDesired > 3.14159)
        //{
        //    adjustedDesired += -6.2831;
        //}
        //else if (adjustedDesired < -3.14159)
        //{
        //    adjustedDesired += 6.2831;
        //}
        
	//if ((((yaw > (result - 0.785)) && (yaw < (result + 0.785))) || (((yaw > 2.36) && (result < -2.36)) || ((yaw < -2.36) && (result > 2.36)))) && brave == false )
	/*if (fabs(adjustedDesired) < 1.5795 && brave == false && destination == true)
	{
	    ROS_INFO("[ASSERTIVE_BEHAVIOUR] Desired Before: %f", adjustedDesired);
	    result = obstacleAvoider(result);
	    ROS_INFO("[ASSERTIVE_BEHAVIOUR] Desired After: %f", adjustedDesired);
	}*/
	
	return result;
}

float AssertiveBehaviour::obstacleAvoider(float desired)
{
      int laserStartIndex = 0;
      int laserWindowRadius = 20;
      //int laserWindowRadius2 = 50;
      bool clear = true;
      bool triedRight = false;
      float possible = desired;
      float yaw = tf::getYaw(latestPoses.transform.rotation);
      
    bool failure = false;


		float adjustedDesired = desired - yaw;
        if (adjustedDesired > 3.14159)
        {
            adjustedDesired += -6.2831;
        }
        else if (adjustedDesired < -3.14159)
        {
            adjustedDesired += 6.2831;
        }
	laserStartIndex = floor(adjustedDesired / 0.0174532) + 90;
		     
      while (failure == false)
      {
          /*for (int i = (laserStartIndex - laserWindowRadius); (i < (laserStartIndex + laserWindowRadius)) && i < 180; i++)
          {
           // ROS_INFO("[ASSERTIVE_BEHAVIOUR] desired: %f start: %d, laserStartIndex: %d end: %d", desired, i, laserStartIndex, (laserStartIndex + laserWindowRadius));
            if (i < 0)
            {
                i = 0;
            }
            if (obstacle == true)
            {
                if (latestLaserScan[i] < 2.00)
                {
                    clear = false;
                }
            }
            else
            {
                if (latestLaserScan[i] < 0.75)
                {
                    clear = false;
                    obstacle = true;
                }
                
            }
            
          }*/
          
          for (int i = 0; i < 180; i++)
          {
            if (latestLaserScan[i] < (0.75 * (1 - (fabs(laserStartIndex - i) / 180))))
            {
                clear = false;
                
            }
          }
          /*for (int i = (laserStartIndex - laserWindowRadius2); (i < (laserStartIndex + laserWindowRadius2)) && i < 180; i++)
          {
            if (i < 0)
            {
                i = 0;
            }
            if (latestLaserScan[i] < 0.5)
            {
                clear = false;
            }
            
          }*/
          /*You found an open way forward!*/
          if (clear == true)
          {
             
             triedRight = false;
             obstacle = false;
             setLights(startupLights);   
              return possible;
          }
          /*If this laser window was not clear, try shifting your angle and search again*/
          else
          {
            clear = true;
            audio_cmd.data = 2;
            audio_pub.publish(audio_cmd);
            ROS_INFO("[ASSERTIVE_BEHAVIOUR] SEARCHING");
            if (triedRight == false)
            {
                //ROS_INFO("[ASSERTIVE_BEHAVIOUR] SEARCH RIGHT");
                laserStartIndex += 10;
                possible += 0.17452;
                if (possible > 3.14159)
                {
                    possible += -6.2831;
                }
                
            }
            else
            {
                //ROS_INFO("[ASSERTIVE_BEHAVIOUR] SEARCH LEFT");
                laserStartIndex -= 10;
                possible -= 0.17452;
                
                
                if (possible < -3.14159)
                {
                    possible += 6.2831;
                }
            }
          }
          /*You found no opening to the right of your desired angle, so try the left.*/
          //if (triedRight == false && laserStartIndex + laserWindowRadius > 180)
          if (triedRight == false && laserStartIndex > 180)
          {
            //ROS_INFO("[ASSERTIVE_BEHAVIOUR] TRIED RIGHT");
            triedRight = true;
            
          }
          /*You've searched everywhere and found no opening! Panic!*/
          else if (triedRight == true && laserStartIndex - laserWindowRadius < 0)
          {
            ROS_INFO("[ASSERTIVE_BEHAVIOUR] FAILURE");
            audio_cmd.data = loseFightSound;
            audio_pub.publish(audio_cmd);
            setLights(loseFightLights);
            failure = true;
            
             move_cmd.linear.x = -0.1;
			 move_cmd.angular.z = 0.5;
		    cmd_vel_pub.publish(move_cmd);
            
            //panicking = true;
            //timer = ros::Time::now();
          }
      }
      /*If you made it here, you found no way forward, so the panic variable was set and you'll be doing that instead.*/
      //ROS_INFO("[ASSERTIVE_BEHAVIOUR] STUCK");
      /*Reset this so next time you'll try right again*/
      triedRight = false;
      obstacle = false;
      return -4;
}

void AssertiveBehaviour::laserTest()
{

        float desiredX;
        float desiredY;
        float desiredAngle = 0;
        float yaw = tf::getYaw(latestPoses.transform.rotation);
        desiredX = goalX;
        desiredY = goalY;
        desiredAngle = getDesiredAngle(desiredX, desiredY, latestPoses.transform.translation.x, latestPoses.transform.translation.y, true);
        
        float adjustedDesired = desiredAngle - yaw;
        if (adjustedDesired > 3.14159)
        {
            adjustedDesired += -6.2831;
        }
        else if (adjustedDesired < -3.14159)
        {
            adjustedDesired += 6.2831;
        }
        ROS_INFO("[ASSERTIVE_BEHAVIOUR] Yaw: %f, Desired Angle: %f, Adjusted Desired: %f", yaw, desiredAngle, adjustedDesired);
        //if (((yaw > (desiredAngle - 0.785)) && (yaw < (desiredAngle + 0.785))) || (((yaw > 2.36) && (desiredAngle < -2.36)) || ((yaw < -2.36) && (desiredAngle > 2.36))))
        if (fabs(adjustedDesired) < 1.5707)
	    {
	        ROS_INFO("[ASSERTIVE_BEHAVIOUR] Desired angle is ahead");
	    }
	    else
	    {
	        ROS_INFO("[ASSERTIVE_BEHAVIOUR] Desired angle is behind");
	    }
}


/*By using your pose and the goal location pose, this function will determine the velocity command it should next issue to continue a normal journey.*/
/*TODO: This is using quaternions since I'm assuming vicon, might need to adjust.*/
void AssertiveBehaviour::waypointing()
{
	if (viconMode == true || stageMode == true)
	{
		float desiredX;
		float desiredY;
		float desiredAngle = 0;
		/*Since the vicon bridge is giving us quaternions, we'll want to get our own yaw back out of it.*/
		//float yaw = tf::getYaw(latestPoses.poses[0].orientation);
		float yaw;		
		if (viconMode == true)
		{
			yaw = tf::getYaw(latestPoses.transform.rotation);
			if (fabs(latestPoses.transform.translation.x - desiredX) < 0.2 && fabs(latestPoses.transform.translation.y - desiredY) < 0.2)
			{
		    		ROS_INFO("[ASSERTIVE_BEHAVIOUR] FLIP DIRECTION");
				returnTrip = !returnTrip;
			}
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
			desiredAngle = getDesiredAngle(desiredX, desiredY, latestPoses.transform.translation.x, latestPoses.transform.translation.y, true);
		}
		else
		{
			yaw = tf::getYaw(stagePose.pose.pose.orientation);
			if (fabs(stagePose.pose.pose.position.x - desiredX) < 0.2 && fabs(stagePose.pose.pose.position.y - desiredY) < 0.2)
			{
		    		ROS_INFO("[ASSERTIVE_BEHAVIOUR] FLIP DIRECTION");
				returnTrip = !returnTrip;
			}
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
			desiredAngle = getDesiredAngle(desiredX, desiredY, stagePose.pose.pose.position.x, stagePose.pose.pose.position.y, true);
		}		
		
		
		
		/*Check if you've arrived at your destination and should switch goals*/
		
		



		/*Now we calculate the yaw we'd want from our current position to be driving toward the goal.*/
			
			/*TODO this messy spot might be wrong*/
			float adjustedDesired = desiredAngle - yaw;
			//if (desiredAngle < -3.5)
			//{
			    /*The laser test found the desired angle to be unreachable so it's already turning you*/
			//}
			
			//else
			//{
			    
				if (adjustedDesired > 3.14159)
				{
					adjustedDesired += -6.2831;
			    	}
				else if (adjustedDesired < -3.14159)
				{
					adjustedDesired += 6.2831;
				}
			//}
			
		    
		    /*
		
		    if ((yaw > (desiredAngle - 0.20)) && (yaw < (desiredAngle + 0.20)) && (driving == true))

		    {
			
			    move_cmd.linear.x = 0.3;
			    move_cmd.angular.z = 0.0;
		    }
	
		    else if ((((yaw > 2.9) && (desiredAngle < -2.9)) || ((yaw < -2.9) && (desiredAngle > 2.9))) && (driving == true))
		    {
			    move_cmd.linear.x = 0.3;
			    move_cmd.angular.z = 0.0;
		    }*/
	
		    /*
		    else if ((yaw > (desiredAngle - 0.15)) && (yaw < (desiredAngle + 0.15)) && (driving == false))

		    {
			    move_cmd.linear.x = 0.3;
			    move_cmd.angular.z = 0.0;
			    driving = true;
		    }

		    else if ((((yaw > 3) && (desiredAngle < -3)) || ((yaw < -3) && (desiredAngle > 3))) && (driving == false))
		    {
			    move_cmd.linear.x = 0.3;
			    move_cmd.angular.z = 0.0;
			    driving = true;
		    }*/
		    /*If we get here then the difference between our yaw and desired angle is too great, moving or not, seam or no seam, so start turning*/
		    if (fabs(adjustedDesired) < 0.3 && driving == true)
		    {
			move_cmd.linear.x = 0.3;
			    move_cmd.angular.z = 0.0;
		    }
		    else if (fabs(adjustedDesired) < 0.2 && driving == false)
		    {
			    driving = true;
			    move_cmd.linear.x = 0.3;
				move_cmd.angular.z = 0.0;
		    }
		    else
		    {
			move_cmd.linear.x = 0.0;
			move_cmd.angular.z = 0.5 * adjustedDesired;
			    driving = false;
		    }
		    cmd_vel_pub.publish(move_cmd);
	}
	else
	{
		
		if (amclReceived == true)
		{
			
			if (returnTrip == false && firstGoal == true)
			{
				//if (doorReached == false && firstGoal == true)
				if (doorReached == false)
				{
					float yawDiff = tf::getYaw(amclPose.pose.pose.orientation) - goalDoorYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					//ROS_INFO("[ASSERTIVE_BEHAVIOUR] yawDiff: %f, goalYaw: %f, poseYaw: %f", yawDiff, goalYaw, tf::getYaw(amclPose.pose.pose.orientation));
					//if (((fabs(amclPose.pose.pose.position.x - goalDoorX) < 0.4 && fabs(amclPose.pose.pose.position.y - goalDoorY) < 0.4) && (fabs(yawDiff) < 0.3)) || firstGoal == false)
					if (((fabs(amclPose.pose.pose.position.x - goalDoorX) < 0.4 && fabs(amclPose.pose.pose.position.y - goalDoorY) < 0.4) && (fabs(yawDiff) < 0.3)))
					{
						/*if(unpaused == false)
						{
							listeningForUnpause = true;
							move_cmd.linear.x = 0.0;
							move_cmd.angular.z = 0.0;
							cmd_vel_pub.publish(move_cmd);
						}
						else
						{*/
							listeningForUnpause = false;
							unpaused = false;
							//tf::Quaternion::Quaternion(startYaw,0,0);
							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = goalX;
							tmpPoint.y = goalY;
							tmpPoint.z = 0.0;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(goalYaw);
							tmpPose.position = tmpPoint;
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;
							goal_pub.publish(goal_cmd);
							setLights(backToNormalLights);
							doorReached = true;
							//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Finished goalDoor, now heading to goal at %f %f", goalX, goalY);
						//}
					}
				}

				else
				{
			
					float yawDiff = tf::getYaw(amclPose.pose.pose.orientation) - goalYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					//ROS_INFO("[ASSERTIVE_BEHAVIOUR] yawDiff: %f, goalYaw: %f, poseYaw: %f", yawDiff, goalYaw, tf::getYaw(amclPose.pose.pose.orientation));
					//if (((fabs(amclPose.pose.pose.position.x - goalX) < 0.4 && fabs(amclPose.pose.pose.position.y - goalY) < 0.4) && (fabs(yawDiff) < 0.3)) || firstGoal == false)
					if (((fabs(amclPose.pose.pose.position.x - goalX) < 0.4 && fabs(amclPose.pose.pose.position.y - goalY) < 0.4) && (fabs(yawDiff) < 0.3)))
					{
						if(unpaused == false)
						{

							listeningForUnpause = true;
							move_cmd.linear.x = 0.0;
							move_cmd.angular.z = 0.0;
							cmd_vel_pub.publish(move_cmd);
						}
						else
						{
							listeningForUnpause = false;
							unpaused = false;
							//tf::Quaternion::Quaternion(startYaw,0,0);
							//firstGoal = true;
							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = startDoorX;
							tmpPoint.y = startDoorY;
							tmpPoint.z = 0.0;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(startDoorYaw);
							tmpPose.position = tmpPoint;
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;
							goal_pub.publish(goal_cmd);
							returnTrip = true;
							aggression = 10 - aggression;
							setLights(backToNormalLights);
							doorReached = false;
							
							//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Finished goal, now heading to startDoor at %f %f", startDoorX, startDoorY);
						}
					}
				}
			}
			else
			{

				if (doorReached == false)
				{
					float yawDiff = tf::getYaw(amclPose.pose.pose.orientation) - startDoorYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					//ROS_INFO("[ASSERTIVE_BEHAVIOUR] yawDiff: %f, goalYaw: %f, poseYaw: %f", yawDiff, startYaw, tf::getYaw(amclPose.pose.pose.orientation));
					if ((fabs(amclPose.pose.pose.position.x - startDoorX) < 0.4 && fabs(amclPose.pose.pose.position.y - startDoorY) < 0.4) &&  (fabs(yawDiff) < 0.3))
					{
						/*if(unpaused == false)
						{

							listeningForUnpause = true;
							move_cmd.linear.x = 0.0;
							move_cmd.angular.z = 0.0;
							cmd_vel_pub.publish(move_cmd);
						}
						else
						{*/
							listeningForUnpause = false;
							unpaused = false;
							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = startX;
							tmpPoint.y = startY;
							tmpPoint.z = 0.0;
							tmpPose.position = tmpPoint;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(startYaw);
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;
							goal_pub.publish(goal_cmd);
							setLights(backToNormalLights);
							doorReached = true;
							//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Finished startDoor, now heading to start at %f %f", startX, startY);
						//}
					}
				}
				else
				{
					float yawDiff = tf::getYaw(amclPose.pose.pose.orientation) - startYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					//ROS_INFO("[ASSERTIVE_BEHAVIOUR] yawDiff: %f, goalYaw: %f, poseYaw: %f", yawDiff, startYaw, tf::getYaw(amclPose.pose.pose.orientation));
					if ((fabs(amclPose.pose.pose.position.x - startX) < 0.4 && fabs(amclPose.pose.pose.position.y - startY) < 0.4) &&  (fabs(yawDiff) < 0.3))
					{
						if(unpaused == false)
						{
							listeningForUnpause = true;
							move_cmd.linear.x = 0.0;
							move_cmd.angular.z = 0.0;
							cmd_vel_pub.publish(move_cmd);
						}
						else
						{
							firstGoal = true;
							listeningForUnpause = false;
							unpaused = false;
							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = goalDoorX;
							tmpPoint.y = goalDoorY;
							tmpPoint.z = 0.0;
							tmpPose.position = tmpPoint;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(goalDoorYaw);
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;
							goal_pub.publish(goal_cmd);
							returnTrip = false;
							aggression = 10 - aggression;
							setLights(backToNormalLights);
							doorReached = false;
							//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Finished start, now heading to goalDoor at %f %f", goalDoorX, goalDoorY);
						}
					}
				}
			}
		}
		else if (amclReceived == false)
		{
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] amcl pose not received");
		}
	}

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

/*When this behaviour is being used with the vicon, this function detects if there is a human or robot subject in the robot's way using their vicon poses. This is an alternative to a sensor-mediated version.*/
/*TODO: Even if I'm not using vicon data to control, I may want to be logging it all.*/
void AssertiveBehaviour::viconSubjectAhead()
{
    float angleToSubject = getDesiredAngle(latestSubjectPoses.transform.translation.x, latestSubjectPoses.transform.translation.y, latestPoses.transform.translation.x, latestPoses.transform.translation.y, false);
    float yaw = tf::getYaw(latestPoses.transform.rotation);
    
    /*Take your pose and orientation compare to the pose of every other tracked subject and if one is detected then set that detection to true*/
    if (fabs(latestPoses.transform.translation.x - latestSubjectPoses.transform.translation.x) < 1.0 
    && fabs(latestPoses.transform.translation.y - latestSubjectPoses.transform.translation.y) < 1.0 
    && ((yaw > (angleToSubject - 0.25)) && (yaw < (angleToSubject + 0.25)) || (((yaw > 2.9) && (angleToSubject < -2.9)) 
    || ((yaw < -2.9) && (angleToSubject > 2.9)))))
    {
        //ROS_INFO("[ASSERTIVE_BEHAVIOUR] SUBJECT AHEAD");
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

	/*if(humanReceived == true)
	{
		subjectDetected = false;
		//humanGrid.info.origin.
		for(int i = 0; i < humanMaximaArray.poses.size(); i++)
		{
			geometry_msgs::Point robotPosition;
			robotPosition.x = 0;
			robotPosition.y = 0;
			robotPosition.z = 0;
			float angleToSubject = getDesiredAngle(humanMaximaArray.poses[i].position.x, humanMaximaArray.poses[i].position.y, 0, 0, false);
			float distanceToSubject = sqrt(pow((humanMaximaArray.poses[i].position.x - robotPosition.x), 2) + pow((humanMaximaArray.poses[i].position.y - robotPosition.y), 2));
			
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Angle to subject: %f, distance to subject: %f", angleToSubject, distanceToSubject);
			if(distanceToSubject < 2.0 && (angleToSubject > -0.3 && angleToSubject < 0.3))
			{
				subjectDetected = true;
			}
		}
	}*/
	if(scrubbedScanReceived == true)
	{
		int counter = 0;
		subjectDetected = false;
		
		for(int i = 20; i < scrubbedScan.ranges.size() - 20; i++)
		{
			/*if (scrubbedScan.ranges[i] != 0)
			{
				counter++;
			}*/
			float allowedDistance = 0.75 + (1.00 - (1.00*(fabs(90 - i)/90)));
			if (scrubbedScan.ranges[i] < allowedDistance && scrubbedScan.ranges[i] != 0.00)
			{
				//ROS_INFO("[ASSERTIVE_BEHAVIOUR]allowed distance: %f detection at: %f detection laser is: %d", allowedDistance, scrubbedScan.ranges[i], i);
				counter++;
				//TODO: MOVE THIS TO THE FIGHT AFTER THE 0.75s WAIT
				/*if (navigating == true && (scrubbedScan.ranges[i] - allowedDistance) < distInitial && counter >= toleranceThreshold)
				{
					distInitial = scrubbedScan.ranges[i] - allowedDistance;
				}*/
			}
		}
		if (counter >= toleranceThreshold)
		{
			subjectDetected = true;
			//ROS_INFO("[ASSERTIVE_BEHAVIOUR] SUBJECT DETECTED: %d", counter);
		}
	}
	else
	{
		subjectDetected = false; 
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] No human detections received");
	}


    }
}


/*A simple function for testing the clearance behind the robot, to be called before backing up
TODO: Replace this with sliding box*/
void AssertiveBehaviour::reverseClearance()
{
    if (sonarReceived == true)
    {
        if(latestSonarScan[8] > 1.0 && latestSonarScan[9] > 1.0 && latestSonarScan[10] > 1.0)
        {
            behindRightClear = true;
        }
        else
        {
            behindRightClear = false;
        }
        if(latestSonarScan[13] > 1.0 && latestSonarScan[14] > 1.0 && latestSonarScan[15] > 1.0)
        {
            behindLeftClear = true;
        }
        else
        {
            behindLeftClear = false;
        }
        if(latestSonarScan[11] > 1.0 && latestSonarScan[12] > 1.0)
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
	behindRightClear = true;
	behindLeftClear = true;
	behindMiddleClear = true;
    	ROS_INFO("[ASSERTIVE_BEHAVIOUR] No sonar received");
    }
}


/*When a fight is initiated, the robot remains in this state until its forward interlocutor is cleared. The robot will either be unsure, or else if it decides the time is right it will advance, or else if it is being advanced upon it will back up.*/
void AssertiveBehaviour::fightingBehaviour()
{
    //ROS_INFO("[ASSERTIVE_BEHAVIOUR] FIGHTING");
    /*TODO: Set good cmd_vel values when retreating*/
    /*First check if you've lost the fight and if so get out of the way*/
	ros::Time tempTime;
	if (stageMode == false)
		{
			tempTime = ros::Time::now();
		}
		else
		{
			tempTime = clockTime.clock;
		}
    if (defeat == true)
    {
        
	//Idea: compare your turn angle to the one taken when you lost, if yawdiff is greater than half then stop moving. Repeat 

	float yawDiff = tf::getYaw(amclPose.pose.pose.orientation) - angleAtDefeat;
	if (yawDiff > 3.14159)
	{
		yawDiff += -6.2831;
	}
	else if (yawDiff < -3.14159)
	{
		yawDiff += 6.2831;
	}
	


       subjectAhead();
	
	if (unwinding == false)
	{


		//if ( yawDiff < 1.570795 && yawDiff > -1.570795)
		if (yawDiff < 1.0 && yawDiff > -1.0 && subjectDetected == true)
		{
			reverseClearance();
	
			if (behindRightClear == true)
			{
			    move_cmd.linear.x = -0.3;
			    move_cmd.angular.z = 0.5;
			}
			else if (behindLeftClear == true)
			{
			    move_cmd.linear.x = -0.3;
			    move_cmd.angular.z = -0.5;
			}
			else if (behindMiddleClear == true)
			{
			    move_cmd.linear.x = -0.3;
			    move_cmd.angular.z = 0.0;
			}
			else 
			{
			    /*You're helpless! Hopefully they get out of your way?*/
			    move_cmd.linear.x = 0.0;
			    move_cmd.angular.z = -0.5;
			    //ROS_INFO("[ASSERTIVE_BEHAVIOUR] STUCK RETREATING");
			}
		}
		else
		{
			move_cmd.linear.x = 0.0;
			move_cmd.angular.z = 0.0;
		}
       		cmd_vel_pub.publish(move_cmd);


	       if (subjectDetected == true)
	       {
			timer = tempTime;
	       }
	       else if (subjectDetected == false && (timer + ros::Duration(1.5) < tempTime))
	       {
		    unwinding = true;
			winner.data = true;
			winner_pub.publish(winner);
	       }   
	
	}
	else
	{
		/*if (subjectDetected == true)
		{
			
			navigating = false;
		    	move_cmd.linear.x = -0.5;
		    	move_cmd.angular.z = 0.0;
		    	cmd_vel_pub.publish(move_cmd);            
		    	fighting = true;
		    	timer = tempTime;
		    	audio_cmd.data = fightStartSound;
		    	audio_pub.publish(audio_cmd);
		    	setLights(fightStartLights);

			unwinding = false;
			distInitial = 10;
			defeat = false;
		}*/
		
		if (yawDiff > 0.35 && emergencyPause.data == false)
		{
			move_cmd.linear.x = 0.3;
		    	move_cmd.angular.z = -0.5;
		}
		else if (yawDiff < -0.35 && emergencyPause.data == false)
		{
			move_cmd.linear.x = 0.3;
		    	move_cmd.angular.z = 0.5;
		}
		else 
		{
			if (emergencyPause.data == true)
			{
				ROS_INFO("[ASSERTIVE_BEHAVIOUR] Abandoning unwinding: emergency stop triggered.");
			}
			move_cmd.linear.x = 0.0;
		    	move_cmd.angular.z = 0.0;
			cmd_vel_pub.publish(move_cmd);
			audio_cmd.data = backToNormalSound;
		    audio_pub.publish(audio_cmd);
		    setLights(backToNormalLights);
		    defeat = false;
		    fighting = false;
		    navigating = true;
			unwinding = false;
			winner.data = false;
			winner_pub.publish(winner);
		}
		cmd_vel_pub.publish(move_cmd);
	}
    }

    else /*Fight ongoing*/
    {
 
        float distCurrent;

	


        //ROS_INFO("[ASSERTIVE_BEHAVIOUR] FIGHTING");
        if ((timer + ros::Duration(0.75) > tempTime))
        {
		subjectAhead();
		if (subjectDetected == false)
		{
			setLights(backToNormalLights);
			fighting = false;
		    	navigating = true;
                	fightStarting = false;
			timer = tempTime;
		}
		else
		{
            		move_cmd.linear.x = -0.2;
			//move_cmd.linear.x = 0.0;
            		move_cmd.angular.z = 0.0;
            		cmd_vel_pub.publish(move_cmd);
		}
        }
        else
        {
		if (fightSoundPlayed == false)
		{
			/*int counter = 0;
			distInitial = 0;
			for(int i = 20; i < scrubbedScan.ranges.size() - 20; i++)
			{
				if (scrubbedScan.ranges[i] != 0)
				{
					distInitial += scrubbedScan.ranges[i];
					counter++;
				}	
			}
			if (counter > 0)
			{
				distInitial = distInitial/counter;
			}
			else
			{
				distInitial = 1.0;
			}
			ROS_INFO("distInitial: %f", distInitial);*/
			audio_cmd.data = fightStartSound;
            		audio_pub.publish(audio_cmd);
			fightSoundPlayed = true;
		}
            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = 0.0;
            cmd_vel_pub.publish(move_cmd);
            
            if (viconMode == true)
            {

		if (fightStarting == false)
            	{
                	fightStarting = true;
               
                    initialX = latestSubjectPoses.transform.translation.x;
                    initialY = latestSubjectPoses.transform.translation.y;
                
                    
                
            	}

            		/*How far were you from the subject at the start of the fight?*/
                    distInitial = sqrt(pow((latestPoses.transform.translation.x - initialX), 2) + pow((latestPoses.transform.translation.y - initialY), 2));
                    /*How far are you now?*/
                    distCurrent = sqrt(pow((latestPoses.transform.translation.x - latestSubjectPoses.transform.translation.x), 2) + pow((latestPoses.transform.translation.y - 			   latestSubjectPoses.transform.translation.y), 2));
            }
            else
            {
                distCurrent = 10;
		if(scrubbedScanReceived == true)
		{
			//int counter = 0;
			
			for(int i = 20; i < scrubbedScan.ranges.size() - 20; i++)
			{
				float allowedDistance = 0.75 + (1.00 - (1.00*(fabs(90 - i)/90)));

				if (distCurrent > scrubbedScan.ranges[i] - allowedDistance && scrubbedScan.ranges[i] != 0)
				{
					distCurrent = scrubbedScan.ranges[i] - allowedDistance;
				}
				/*if (scrubbedScan.ranges[i] != 0)
				{
					distCurrent += scrubbedScan.ranges[i];
					counter++;
				}*/	
			}
			/*if (counter > 0)
			{
				distCurrent = distCurrent/counter;
			}
			else
			{
				ROS_INFO("[ASSERTIVE_BEHAVIOUR] No points in scrubbed scan???");
				distCurrent = 1.0;
			}*/
			
		}
		else
		{
			distCurrent = 0.1;
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Can't fight without scrubbed laser");
		}
		if (distCurrent == 10)
		{
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] NO POINTS FOUND?????");
		}



            }
            
            

		//ROS_INFO("[ASSERTIVE_BEHAVIOUR] distCurrent: %f, distInitial: %f, loseDistance: %f, winDistance: %f", distCurrent, distInitial, loseDistance, winDistance);
            
            //if (distInitial > distCurrent + loseDistance) /*If they have moved toward you, you lose*/
		if (distCurrent < loseDistance)
		//TODO: manual_mode
		//if (/*command A pushed*/)
            {
                audio_cmd.data = loseFightSound;
                audio_pub.publish(audio_cmd);
                setLights(loseFightLights);
                timer = tempTime;
                defeat = true;
                fightStarting = false;
		angleAtDefeat = tf::getYaw(amclPose.pose.pose.orientation);
		fightSoundPlayed = false;
		ROS_INFO("LOST: distInitial: %f, distCurrent: %f, loseDistance: %f", distInitial, distCurrent, loseDistance);
		
            }
            //else if(distCurrent > distInitial + winDistance) /*If they have moved away from you, you win!*/
		else if(distCurrent > winDistance)
		//TODO: manual_mode
		//else if (/*command B pushed*/)
            {
                audio_cmd.data = winFightSound;
                audio_pub.publish(audio_cmd);
                setLights(winFightLights);
                timer = tempTime;
                brave = true;
                fighting = false;
                navigating = true;
                fightStarting = false;
		fightSoundPlayed = false;
		winner.data = true;
		winner_pub.publish(winner);
		ROS_INFO("WON: distInitial: %f, distCurrent: %f, winDistance: %f", distInitial, distCurrent, winDistance);
            }
            else if (tempTime - timer >  ros::Duration(aggression)) /*Your patience ran out, so you win! Probably!*/
            	//TODO: manual_mode
		//else if(/*command C pushed*/)
		{
                audio_cmd.data = winFightSound;
                audio_pub.publish(audio_cmd);
                setLights(winFightLights);
                timer = tempTime;
                brave = true;
                fighting = false;
                navigating = true;
                fightStarting = false;
		fightSoundPlayed = false;
		winner.data = true;
		winner_pub.publish(winner);
		ROS_INFO("WON??: distInitial: %f, distCurrent: %f, loseDistance: %f, winDistance: %f", distInitial, distCurrent, loseDistance, winDistance);
            }
            else /*keep waiting*/
            {
                /*Maybe should gradient to red lights, increase hum to signal patience running out?*/
            }
        }
        
    }    

}


/*This behaviour is called if something is within emergency range or some other trapped/deadlocked scenario. One consideration is making the difference between "too close" as mindless obstacle and "too close" as other robot/person, where expressing displeasure at being pushed may be warranted.*/
void AssertiveBehaviour::panickingBehaviour()
{
/*TODO: Fill this in. Stop? Spin? Back away?*/
ROS_INFO("[ASSERTIVE_BEHAVIOUR] PANIC");
/*    move_cmd.linear.x = 0.0;
    move_cmd.angular.z = 0.0;
    cmd_vel_pub.publish(move_cmd);*/

	ros::Time tempTime;
	if (stageMode == false)
	{
		tempTime = ros::Time::now();
	}
	else
	{
		tempTime = clockTime.clock;
	}


    if (tempTime - timer >  ros::Duration(3))
    {
        reverseClearance();
        if (behindRightClear == true)
        {
            move_cmd.linear.x = -0.5;
            move_cmd.angular.z = -0.5;
        }
        else if (behindLeftClear == true)
        {
            move_cmd.linear.x = -0.5;
            move_cmd.angular.z = 0.5;
        }
        else if (behindMiddleClear == true)
        {
            move_cmd.linear.x = -0.5;
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
    else
    {
        audio_cmd.data = backToNormalSound;
                audio_pub.publish(audio_cmd);
                setLights(backToNormalLights);
        panicking = false;
        navigating = true;
    }
    
    
}


/*This is the normal driving behaviour when either no subject is ahead or else you won the fight. Handles driving as well as obstace and subject detection.*/
void AssertiveBehaviour::navigatingBehaviour()
{
	
    if ((((viconMode == true || stageMode == true) && poseReceived == true) || (viconMode == false && stageMode == false)) && laserReceived == true)
    {
	ros::Time tempTime;
	if (stageMode == false)
	{
		tempTime = ros::Time::now();
	}
	else
	{
		tempTime = clockTime.clock;
	}

        subjectAhead();
        /*If someone is newly detected ahead of you, a fight starts*/
        if (subjectDetected == true  && brave == false)
	//TODO: manual_mode
	//if (/*command pushed*/)
        {
		navigating = false;
            //move_cmd.linear.x = -0.35;
		move_cmd.linear.x = 0.0;
            move_cmd.angular.z = 0.0;
            cmd_vel_pub.publish(move_cmd);            
            fighting = true;
            timer = tempTime;
            //moveOrderTimer = tempTime;
            setLights(fightStartLights);
        }
        else if (subjectDetected == true  && brave == true) /*Someone is in front of you but you won a fight so keep pushing*/
        {
		//if(latestSonarScan[3] < 0.5 || latestSonarScan[4] < 0.5)
		/*for(int i = 20; i < scrubbedScan.ranges.size() - 20; i++)
		{
			float allowedDistanceScrubbed = 0.4 + (0.20 - (0.20*(fabs(90 - i)/90)));
			//float allowedDistanceInverse = 0.15 + (0.15 - (0.15*(fabs(90 - i)/90)));
			if (scrubbedScan.ranges[i] < allowedDistanceScrubbed && scrubbedScan.ranges[i] != 0.00)
			{
				//ROS_INFO("[ASSERTIVE_BEHAVIOUR]In my wayyyyyyyyyyy");
				emergencyPause = true;
			}
		}*/
		if (emergencyPause.data == true)
		{
			
			
			if (timer + ros::Duration(10 - aggression) < tempTime)
			{
				
				brave = false;
				navigating = false;
				audio_cmd.data = loseFightSound;
                		audio_pub.publish(audio_cmd);
                		setLights(loseFightLights);
                		timer = tempTime;
                		defeat = true;
				fighting = true;
                		fightStarting = false;
				angleAtDefeat = tf::getYaw(amclPose.pose.pose.orientation);
				fightSoundPlayed = false;
				winner.data = false;
				winner_pub.publish(winner);

			}
			else
			{
				move_cmd.linear.x = 0.0;
            			move_cmd.angular.z = 0.0;
            			cmd_vel_pub.publish(move_cmd); 
			}
		}
		else
		{
                	timer = tempTime;
                	waypointing();
		}
                //ROS_INFO("[ASSERTIVE_BEHAVIOUR] WINNING");
        }
        else if (subjectDetected == false  && brave == true) /*you won a fight but there's no longer someone in front of you*/
        {
            if (timer + ros::Duration(2) < tempTime) /*If they haven't been re-detected in front of you in 3 seconds stop being brave*/
            {
               // ROS_INFO("[ASSERTIVE_BEHAVIOUR] WON");
                audio_cmd.data = backToNormalSound;
                audio_pub.publish(audio_cmd);
                setLights(backToNormalLights);
                brave = false;
		winner.data = false;
		winner_pub.publish(winner);
            }
            waypointing();
        } /*Normal driving*/
        else
        {
            //ROS_INFO("[ASSERTIVE_BEHAVIOUR] Waypointing");
            waypointing();
        }
        
    }
    else
    {
        ROS_INFO("[ASSERTIVE_BEHAVIOUR] Vicon: %d Laser: %d.", poseReceived, laserReceived);
    }
}

/*One run of the loop, picks which behaviour to do depending on what state the robot is in. The different behaviours control state transitions based on their own criteria, this is just where the program reassesses which one to apply.*/
void AssertiveBehaviour::spinOnce() {
	ros::Rate rate(loopHz);
	/*static float count = 0;
	printf("current rate: %f\n", count++ / (10* (ros::Time::now().toSec() - beginTimer.toSec())));*/
    if ((((viconMode == true || stageMode == true) && poseReceived == true) || (viconMode == false)) && laserReceived == true && scrubbedScanReceived == true)
    {
		
	    if((timer + ros::Duration(5) < ros::Time::now()) && firstTime == false)
	    {
		audio_cmd.data = startupSound;
		audio_pub.publish(audio_cmd);
		setLights(startupLights);
		firstTime = true;
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] Let's get started!");
	    } 
	     
	    else if (firstTime == true){    
		if(panicking)
		{
		    //panickingBehaviour();
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Panic");
		}
		else if(fighting)
		{
		    fightingBehaviour();
			//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Fight");

		}
		else if(navigating)
		{
		    
		    navigatingBehaviour();
			//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Navigate");
		}
		else
		{
		    ROS_INFO("[ASSERTIVE_BEHAVIOUR] Stateless");
		}
	
	
		}
		else
		{}
	  	
	}
	else
	{
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] vicon mode: %d, stage mode: %d, pose received: %d, laser received: %d, scrubbed scan received: %d, amcl pose received: %d", viconMode, stageMode, poseReceived, laserReceived, scrubbedScanReceived, amclReceived);
	}
	rate.sleep();
	ros::spinOnce();
}

/*Gets called by main, runs all the time at the given rate.*/
void AssertiveBehaviour::spin() {
  //ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
	//rate.sleep();
  }
}
