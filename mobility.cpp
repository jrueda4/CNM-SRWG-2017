//#INCLUDES
//--------------------------------------------
#include <ros/ros.h>
#include <string>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// Include Controllers
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

using namespace std;

// Variables
//--------------------------------------------
random_numbers::RandomNumberGenerator* rng;     // Random number generator... should be DELETED later

// STATE MACHINE STATE CONSTANTS (for mobility SWITCH)
//--------------------------------------------
#define STATE_MACHINE_TRANSFORM 0
#define STATE_MACHINE_ROTATE 1
#define STATE_MACHINE_SKID_STEER 2
#define STATE_MACHINE_PICKUP 3
#define STATE_MACHINE_DROPOFF 4

int stateMachineState = STATE_MACHINE_TRANSFORM; //stateMachineState keeps track of current state in mobility state machine

const unsigned int mapHistorySize = 500;        // How many points to use in calculating the map average position

//GEOMETRY_MSG::POSE2D CLASS OBJECTS            //x, y, theta public variables (vectors)
//--------------------------------------------
geometry_msgs::Pose2D currentLocation;          //current location of robot
geometry_msgs::Pose2D currentLocationMap;       //current location on MAP
geometry_msgs::Pose2D currentLocationAverage;   //???
geometry_msgs::Pose2D goalLocation;             //location to drive to
//geometry_msgs::Pose2D clusterLocation;            //for the BUDDY SYSTEM ++++++++++++

geometry_msgs::Pose2D centerLocation;           //location of center location
geometry_msgs::Pose2D centerLocationMap;        //location of center on map
geometry_msgs::Pose2D centerLocationOdom;       //location of center ODOM

geometry_msgs::Pose2D mapLocation[mapHistorySize]; //An array in which to store map positions

std_msgs::String msg;                           //std_msgs shares current STATE_MACHINE STATUS in mobility state machine
geometry_msgs::Twist velocity;                  //???

//Do not know what the map and odom locations do ATM - JMS

//Controller Class Objects
//--------------------------------------------
PickUpController pickUpController;
DropOffController dropOffController;
SearchController searchController;

int currentMode = 0;
float mobilityLoopTimeStep = 0.1;               // time between the mobility loop calls
float status_publish_interval = 1;
float killSwitchTimeout = 10;
bool targetDetected = false;                    //for target detection    (seen a target)
bool targetCollected = false;                   //for target collection   (picked up a target)
bool avoidingObstacle = false;

// Set true when the target block is less than targetDist so we continue
// attempting to pick it up rather than switching to another block in view.
bool lockTarget = false;

// Failsafe state. No legitimate behavior state. If in this state for too long
// return to searching as default behavior.
bool timeOut = false;

// Set to true when the center ultrasound reads less than 0.14m. Usually means
// a picked up cube is in the way.
bool blockBlock = false;

// Set true when we are insie the center circle and we need to drop the block,
// back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

// used for calling code once but not in main
bool init = false;

// used to remember place in mapAverage array
int mapCount = 0;

//Function Calls
//--------------------------------------------

//sets speed
void sendDriveCommand(double linearVel, double angularVel);

void openFingers();                             // Open fingers to 90 degrees
void closeFingers();                            // Close fingers to 0 degrees
void raiseWrist();                              // Return wrist back to 0 degrees
void lowerWrist();                              // Lower wrist to 50 degrees
void mapAverage();                              // constantly averages last 100 positions from map

//PUBLISHER/SUBSCRIBER/TIMER
//--------------------------------------------

// Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;
ros::Publisher buddyIDPublish;
//ros::Publisher buddyLocationPub;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;
ros::Subscriber buddyIDSubscriber;
//ros::Subscriber buddyLocationSub;

// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;

time_t timerStartTime;                          // records time for delays in sequanced actions, 1 second resolution.

unsigned int startDelayInSeconds = 1;           // An initial delay to allow the rover to gather enough position data to
                                                // average its location.
float timerTimeElapsed = 0;

char host[128];
string publishedName;
char prev_state_machine[128];

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);
void buddyIDcallBack(const std_msgs::Float32& msgID);

//buddy system+++++++++++++++++++++++++++++++++++++++++++
bool SwarmieID = 0;

//CNM Code Follows:
//--------------------------------------------


//Central collection point has been seen (aka the nest)
bool centerSeen = false;
bool cnmHasCenterLocation = false;

//Manipulative ORIGINAL FILE Variables
float searchVelocity = 0.4; // meters/second  ORIGINALLY .2

//First Boot Boolean
bool cnmFirstBootProtocol = true;

//Variables under MobilityStateMachine
float rotateOnlyAngleTolerance = 0.2;       //jms chnaged from .4
int returnToSearchDelay = 5;

//Variables for Obstacle Avoidance
bool cnmAvoidObstacle = false;
bool cnmSeenAnObstacle = false;

//Obstacle Avoidance Timer and Duration
ros::Timer cnmObstacleAvoidanceTimer;
ros::Duration cnmObstacleTimerTime(10);

void InitComp();                            //INIT COMPONENTS (Builds map, sets Pose 2D Objects defaults)

void CNMFirstBoot();                        //Code for robot to run on initial switch to autonomous mode

bool CNMTransformCode();                    //A function for the Transform segment in Mobility State Machine
                                                //	- returns true only if it needs to return
bool CNMPickupCode();                       //A function for PickUpController in Mobility State Machine
                                                //	- returns false if it needs to break
bool CNMRotateCode();                       //A function for the Rotate Mobility State Machine Code

void CNMSkidSteerCode();                    //A function with all the skid steer mobility code


//Timer Functions/Callbacks
void CNMAvoidance(const ros::TimerEvent& event);    //Timer Function(when timer fires, it runs this code)

//MAIN
//--------------------------------------------
int main(int argc, char **argv)
{

    gethostname(host, sizeof(host));
    string hostname(host);

    InitComp();

    // instantiate random number generator
    rng = new random_numbers::RandomNumberGenerator();

    if (argc >= 2)
    {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName
            << "!  Mobility turnDirectionule started." << endl;
    }
    else
    {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    // Register the SIGINT event handler so the node can shutdown properly
    signal(SIGINT, sigintEventHandler);

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
    mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);
    buddyIDSubscriber = mNH.subscribe((publishedName + "/msgID"), 1000, buddyIDcallBack);//buddySystem +++++++++++++++++++++
//    buddyLocationSub = mNH.subscribe((publishedName + "/clusterLocation"), 1000, &buddyLocationCallBack);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
    driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
    buddyIDPublish = mNH.advertise<std_msgs::Float32>((publishedName + "/msgID"), 1000);// buddySystem+++++++++++++++++++++
//    buddyLocationPub = mNH.advertise<geometry_msgs::Pose2D>((publishedName + "/clusterLocation"), 1000);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);


    //CNM Timer Info
    cnmObstacleAvoidanceTimer = mNH.createTimer(cnmObstacleTimerTime, CNMAvoidance, true);
    cnmObstacleAvoidanceTimer.stop();

    tfListener = new tf::TransformListener();
    std_msgs::String msg;
    msg.data = "Log Started";
    infoLogPublisher.publish(msg);

    stringstream ss;
    ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    timerStartTime = time(0);
    //BUDDY SYSTEM +++++++++++++++++++++++++
    if(SwarmieID == 1 && targetDetected == false)
    {
      std_msgs::String msg;
            msg.data = "I AM buddy";
            infoLogPublisher.publish(msg);
    }
    
    if(SwarmieID == 1 && targetDetected == true)
    {
      std_msgs::String msg;
            msg.data = "I AM finder";
            infoLogPublisher.publish(msg); 
    }
    ros::spin();

    return EXIT_SUCCESS;
}


// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void mobilityStateMachine(const ros::TimerEvent&)
{

    if(cnmFirstBootProtocol)
    {
        cnmFirstBootProtocol = false;

        CNMFirstBoot();
    }

    std_msgs::String stateMachineMsg;

    // calls the averaging function, also responsible for
    // transform from Map frame to odom frame.
    mapAverage();

    // Robot is in automode
    if (currentMode == 2 || currentMode == 3)
    {
        // time since timerStartTime was set to current time
        timerTimeElapsed = time(0) - timerStartTime;

        // init code goes here. (code that runs only once at start of
        // auto mode but wont work in main goes here)
        if (!init)
        {
            if (timerTimeElapsed > startDelayInSeconds)
            {
                // Set the location of the center circle location in the map
                // frame based upon our current average location on the map.
                centerLocationMap.x = currentLocationAverage.x;
                centerLocationMap.y = currentLocationAverage.y;
                centerLocationMap.theta = currentLocationAverage.theta;

                // initialization has run
                init = true;
            }
            else { return; }
        }

        // If no collected or detected blocks set fingers
        // to open wide and raised position.
        if (!targetCollected && !targetDetected)
        {
            // set gripper
            std_msgs::Float32 angle;

            // open fingers
            angle.data = M_PI_2;

            fingerAnglePublish.publish(angle);
            angle.data = 0;

            // raise wrist
            wristAnglePublish.publish(angle);
        }

        // Select rotation or translation based on required adjustment
        switch (stateMachineState)
        {
            // If no adjustment needed, select new goal
        case STATE_MACHINE_TRANSFORM:
        {
            stateMachineMsg.data = "TRANSFORMING";

            if (!CNMTransformCode()) { break; }
            //Purposefully fall through to next case without breaking
        }

        // Calculate angle between currentLocation.theta and goalLocation.theta
        // Rotate left or right depending on sign of angle
        // Stay in this state until angle is minimized
        case STATE_MACHINE_ROTATE:
        {
            stateMachineMsg.data = "ROTATING";

            if(CNMRotateCode()) { break; }

            //Purposefully fall through to next case without breaking
        }

        // Calculate angle between currentLocation.x/y and goalLocation.x/y
        // Drive forward
        // Stay in this state until angle is at least PI/2
        case STATE_MACHINE_SKID_STEER:
        {
            stateMachineMsg.data = "SKID_STEER";

            CNMSkidSteerCode();

            break;
        }

        case STATE_MACHINE_PICKUP:
        {
            stateMachineMsg.data = "PICKUP";

            if(CNMPickupCode()) { return; }

            break;
        }

        case STATE_MACHINE_DROPOFF: {
            stateMachineMsg.data = "DROPOFF";
            break;
        }

        default:
        {
            break;
        }

        } /* end of switch() */
    }
    // mode is NOT auto
    else
    {
        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0)
    {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}

void sendDriveCommand(double linearVel, double angularError)
{
    velocity.linear.x = linearVel,
        velocity.angular.z = angularError;

    // publish the drive commands
    driveControlPublish.publish(velocity);
}

/*************************
* ROS CALLBACK HANDLERS *
*************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{

    // If in manual mode do not try to automatically pick up the target
    if (currentMode == 1 || currentMode == 0) { return; }

    // if a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint)
    {
        float cameraOffsetCorrection = 0.020; //meters;

        centerSeen = false;             //set to false
        double count = 0;
        double countRight = 0;
        double countLeft = 0;

        // this loop is to get the number of center tags
        for (int i = 0; i < message->detections.size(); i++)
        {
            if (message->detections[i].id == 256)
            {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                // checks if tag is on the right or left side of the image
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0)
                {
                    countRight++;
                }

                else
                {
                    countLeft++;
                }

                centerSeen = true;
                cnmHasCenterLocation = true;
                count++;
            }
        }

        //CNM ADDED
        if(centerSeen)
        {
            searchController.setCenterSeen(true);
            geometry_msgs::Pose2D location;

            location = centerLocation;

            location.x = currentLocation.x + (0.3 * cos(currentLocation.theta));
            location.y = currentLocation.y + (0.3 * sin(currentLocation.theta));

            searchController.setCenterLocation(location);
        }

        //---------------------------------------------

        if (centerSeen && targetCollected)
        {
            stateMachineState = STATE_MACHINE_TRANSFORM;
            goalLocation = currentLocation;
        }

        dropOffController.setDataTargets(count, countLeft, countRight);

        // if we see the center and we dont have a target collected
        if (centerSeen && !targetCollected)
        {

            float centeringTurn = 0.15; //radians
            stateMachineState = STATE_MACHINE_TRANSFORM;

            // this code keeps the robot from driving over
            // the center when searching for blocks
            if (right)
            {
                // turn away from the center to the left if just driving
                // around/searching.
                //goalLocation.theta += centeringTurn;
            }
            else
            {
                // turn away from the center to the right if just driving
                // around/searching.
                //goalLocation.theta += centeringTurn;
            }

            searchController.cnmSetRotations(0);
            sendDriveCommand(-.4,0);

            // continues an interrupted search
            goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

            targetDetected = false;
            pickUpController.reset();

            return;
        }

    }
    // end found target and looking for center tags

    // found a target april tag and looking for april cubes;
    // with safety timer at greater than 5 seconds.
    PickUpResult result;

    //if we see an april tag, and don't have a target, and if timer is ok, and we have found the nest
    if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5 && cnmHasCenterLocation == true)
    {
        targetDetected = true;
        
        std_msgs::Float32 msg; //buddySystem +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        msg.data = 1;
        buddyIDPublish.publish(msg);

        // pickup state so target handler can take over driving.
        stateMachineState = STATE_MACHINE_PICKUP;
        result = pickUpController.selectTarget(message);

        std_msgs::Float32 angle;

        if (result.fingerAngle != -1)
        {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1)
        {
            angle.data = result.wristAngle;
            wristAnglePublish.publish(angle);
        }
    }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message)
{
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message)
{
    //if avoiding an obstacle
    if(cnmAvoidObstacle == true)
    {

        goalLocation.theta = currentLocation.theta + (M_PI/2);

        goalLocation.x = currentLocation.x + (0.20 * cos(goalLocation.theta));
        goalLocation.y = currentLocation.y + (0.20 * cos(goalLocation.theta));


        stateMachineState = STATE_MACHINE_ROTATE;

        if(message->data == 0)
        {
            std_msgs::String msg;
            msg.data = "Navigated Obstacle";
            infoLogPublisher.publish(msg);

            // continues an interrupted search
            goalLocation = searchController.continueInterruptedSearch(currentLocation, goalLocation);

            cnmAvoidObstacle = false;
            cnmSeenAnObstacle = false;
        }

    }
    //no matter what we receive from obstacle
    else if ((!targetDetected || targetCollected) && (message->data > 0))
    {

        if (message->data == 1 || message->data == 2 || message->data == 3)
        {
            //start obstacle timer
            cnmObstacleAvoidanceTimer.start();

            //if you see something stop
            sendDriveCommand(0.0, 0.0);

            //if this is the first time we are acknowledging an obstacle
            if(!cnmSeenAnObstacle)
            {
                std_msgs::String msg;
                msg.data = "Obstacle Detected";
                infoLogPublisher.publish(msg);
                cnmSeenAnObstacle = true;
            }

            stateMachineState = STATE_MACHINE_ROTATE;
        }
    }
    //if we saw a target but no longer see one
    else if (cnmSeenAnObstacle && (!targetDetected || targetCollected) && (message->data == 0))
    {
        cnmSeenAnObstacle = false;
        cnmAvoidObstacle = false;

        std_msgs::String msg;
        msg.data = "Obstacle Moved, Continuing Search Pattern";
        infoLogPublisher.publish(msg);

        cnmObstacleAvoidanceTimer.stop();

        // switch to transform state to trigger collision avoidance
        stateMachineState = STATE_MACHINE_ROTATE;
    }

    // the front ultrasond is blocked very closely. 0.14m currently
    if (message->data == 4)
    {
        blockBlock = true;
    }
    else
    {
        blockBlock = false;
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message)
{
    //Get (x,y) location directly from pose
    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
}

void mapHandler(const nav_msgs::Odometry::ConstPtr& message)
{
    //Get (x,y) location directly from pose
    currentLocationMap.x = message->pose.pose.position.x;
    currentLocationMap.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocationMap.theta = yaw;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message)
{
    if (currentMode == 0 || currentMode == 1)
    {
        sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
    }
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
    std_msgs::String msg;
    msg.data = "CNMSRWG17 Online";
    status_publisher.publish(msg);
}


void targetDetectedReset(const ros::TimerEvent& event)
{
    targetDetected = false;

    std_msgs::Float32 angle;
    angle.data = 0;

    // close fingers
    fingerAnglePublish.publish(angle);

    // raise wrist
    wristAnglePublish.publish(angle);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void mapAverage()
{
    // store currentLocation in the averaging array
    mapLocation[mapCount] = currentLocationMap;
    mapCount++;

    if (mapCount >= mapHistorySize)
    {
        mapCount = 0;
    }

    double x = 0;
    double y = 0;
    double theta = 0;

    // add up all the positions in the array
    for (int i = 0; i < mapHistorySize; i++)
    {
        x += mapLocation[i].x;
        y += mapLocation[i].y;
        theta += mapLocation[i].theta;
    }

    // find the average
    x = x / mapHistorySize;
    y = y / mapHistorySize;

    // Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    theta = theta / 100;
    currentLocationAverage.x = x;
    currentLocationAverage.y = y;
    currentLocationAverage.theta = theta;

    // only run below code if a centerLocation has been set by initilization
    if (init)
    {
        // map frame
        geometry_msgs::PoseStamped mapPose;

        // setup msg to represent the center location in map frame
        mapPose.header.stamp = ros::Time::now();

        mapPose.header.frame_id = publishedName + "/map";
        mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
        mapPose.pose.position.x = centerLocationMap.x;
        mapPose.pose.position.y = centerLocationMap.y;
        geometry_msgs::PoseStamped odomPose;
        string x = "";

        try
        {
            //attempt to get the transform of the center point in map frame to odom frame.
            tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
            tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
        }

        catch (tf::TransformException& ex)
        {
            ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
            x = "Exception thrown " + (string)ex.what();
            std_msgs::String msg;
            stringstream ss;
            ss << "Exception in mapAverage() " + (string)ex.what();
            msg.data = ss.str();
            infoLogPublisher.publish(msg);
        }

        // Use the position and orientation provided by the ros transform.
        centerLocation.x = odomPose.pose.position.x; //set centerLocation in odom frame
        centerLocation.y = odomPose.pose.position.y;
    }
}

//CNM Functions Follow
//-----------------------------------

//INITIALIZE COMPONENTS
//-----------------------------------
void InitComp()
{
    //This is original code from main, moved to a utility function.

    //create map
    for (int i = 0; i < 100; i++)
    {
        mapLocation[i].x = 0;
        mapLocation[i].y = 0;
        mapLocation[i].theta = 0;
    }

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocationOdom.x = 0;
    centerLocationOdom.y = 0;
}

//Set of code to run before search pattern starts
//-----------------------------------
void CNMFirstBoot()
{
    //set initial heading 180 degress from initial theta
    goalLocation.theta = currentLocation.theta + M_PI;

    //select position 50 cm from origin before attempting to go into search pattern
    goalLocation.x = 0.5 * cos(goalLocation.theta + M_PI);
    goalLocation.y = 0.5 * sin(goalLocation.theta + M_PI);

    //move robot state to ROTATE because of the new coordinates.
    stateMachineState = STATE_MACHINE_ROTATE;
}

//MOBILITY TRANFORM STATES
//-----------------------------------
bool CNMPickupCode()
{

    PickUpResult result;

    // we see a block and have not picked one up yet
    if (targetDetected && !targetCollected)
    {
        result = pickUpController.pickUpSelectedTarget(blockBlock);
        sendDriveCommand(result.cmdVel, result.angleError);
        std_msgs::Float32 angle;

        if (result.fingerAngle != -1)
        {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1)
        {
            angle.data = result.wristAngle;

            // raise wrist
            wristAnglePublish.publish(angle);
        }

        if (result.giveUp)
        {
            targetDetected = false;
            stateMachineState = STATE_MACHINE_TRANSFORM;
            sendDriveCommand(0, 0);
            pickUpController.reset();
        }

        if (result.pickedUp)
        {
            pickUpController.reset();

            // assume target has been picked up by gripper
            targetCollected = true;
            result.pickedUp = false;
            stateMachineState = STATE_MACHINE_ROTATE;

            goalLocation.theta = atan2(centerLocationOdom.y - currentLocation.y, centerLocationOdom.x - currentLocation.x);

            // set center as goal position
            goalLocation.x = centerLocationOdom.x = 0;
            goalLocation.y = centerLocationOdom.y;

            // lower wrist to avoid ultrasound sensors
            std_msgs::Float32 angle;
            angle.data = 0.8;
            wristAnglePublish.publish(angle);
            sendDriveCommand(0.0, 0);

            return true;
        }
    }
    else
    {
        stateMachineState = STATE_MACHINE_TRANSFORM;
    }

    return false;
}

bool CNMTransformCode()
{

    // If returning with a target
    if (targetCollected && !avoidingObstacle)
    {
        // calculate the euclidean distance between
        // centerLocation and currentLocation
        dropOffController.setCenterDist(hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y));
        dropOffController.setDataLocations(centerLocation, currentLocation, timerTimeElapsed);

        DropOffResult result = dropOffController.getState();

        if (result.timer)
        {
            timerStartTime = time(0);
            reachedCollectionPoint = true;
        }

        std_msgs::Float32 angle;

        if (result.fingerAngle != -1)
        {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1)
        {
            angle.data = result.wristAngle;
            wristAnglePublish.publish(angle);
        }

        if (result.reset)
        {
            timerStartTime = time(0);
            targetCollected = false;
            targetDetected = false;
            lockTarget = false;
            sendDriveCommand(0.0, 0);

            // move back to transform step
            stateMachineState = STATE_MACHINE_TRANSFORM;
            reachedCollectionPoint = false;;
            centerLocationOdom = currentLocation;

            dropOffController.reset();
        }
        else if (result.goalDriving && timerTimeElapsed >= 5)
        {
            goalLocation = result.centerGoal;
            stateMachineState = STATE_MACHINE_ROTATE;
            timerStartTime = time(0);
        }

        // we are in precision/timed driving
        else
        {
            goalLocation = currentLocation;
            sendDriveCommand(result.cmdVel, result.angleError);
            stateMachineState = STATE_MACHINE_TRANSFORM;

            return false;
        }
    }

    //If angle between current and goal is significant
    //if error in heading is greater than 0.4 radians
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) >
        rotateOnlyAngleTolerance)
    {
        stateMachineState = STATE_MACHINE_ROTATE;
    }

    //If goal has not yet been reached drive and maintain heading
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta,
        atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2)
    {
        stateMachineState = STATE_MACHINE_SKID_STEER;
    }

    //Otherwise, drop off target and select new random uniform heading
    //If no targets have been detected, assign a new goal
    else if (!targetDetected && timerTimeElapsed > returnToSearchDelay)
    {
        goalLocation = searchController.search(currentLocation);
    }

    return true;
}

bool CNMRotateCode()
{

    // Calculate the diffrence between current and desired
    // heading in radians.
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

    // If angle > 0.4 radians rotate but dont drive forward.
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance)
    {
        // rotate but dont drive  0.05 is to prevent turning in reverse
        sendDriveCommand(0.05, errorYaw);
        return true;
    }
    else
    {
        // move to differential drive step
        stateMachineState = STATE_MACHINE_SKID_STEER;
        //fall through on purpose.
    }

    return false;
}

void CNMSkidSteerCode()
{
    // calculate the distance between current and desired heading in radians
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);

    // goal not yet reached drive while maintaining proper heading.
    if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2)
    {
        // drive and turn simultaniously
        sendDriveCommand(searchVelocity, errorYaw / 2);
    }
    // goal is reached but desired heading is still wrong turn only
    else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1)
    {
        // rotate but dont drive
        sendDriveCommand(0.0, errorYaw);
    }
    else
    {
        // stop
        sendDriveCommand(0.0, 0.0);
        avoidingObstacle = false;

        // move back to transform step
        stateMachineState = STATE_MACHINE_TRANSFORM;
    }
}

//CNM TIMER FUNCTIONS
//-----------------------------------
void CNMAvoidance(const ros::TimerEvent &event)
{
    
    //select new heading 0.4 radians to the left
    goalLocation.theta = currentLocation.theta + 0.4;
    //select new position 30 cm from current location
    goalLocation.x = currentLocation.x + (0.3 * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (0.3 * sin(goalLocation.theta));
    
    cnmAvoidObstacle = true;

    std_msgs::String msg;
    msg.data = "Obstacle Avoidance Initiated";
    infoLogPublisher.publish(msg);
}

void buddyIDcallBack(const std_msgs::Float32& message)
    {
      SwarmieID = 1;
    }
