#include "SearchController.h"

//CNM VARIABLES
//--------------------------------------
geometry_msgs::Pose2D cnmCenterLocation;


//BOOLEAN TRIGGERS
//--------------------------------------
bool cnmCenterSeen;
bool first;

//PRIMITIVES
//--------------------------------------

//Target Search
int searchLoop;             //int of 0 - 8
int looping;                //int of 0 - 7

double searchDist;
double searchCounter;

//Center Search
int centerLoop;

double centerSearch;

SearchController::SearchController() 
{

    //CNM VARIABLES
    //--------------------------------------
    searchLoop = 0;
    looping = 0;
    searchDist = .2;

    centerLoop = 0;
    centerSearch = .4;

    cnmNumRotations = 0;

    cnmCenterSeen = false;
    first = true;

    rng = new random_numbers::RandomNumberGenerator();

    //searchLoop = rng->uniformInteger(0, 8);                 //random point in loop between 0 and 8
    searchCounter = rng->uniformReal(.90, 1.16);    //random distance from center to start searching

    centerLoop = rng->uniformInteger(0, 8);
    searchDist = .2;

    cnmCenterLocation.x = 0;
    cnmCenterLocation.y = 0;
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) 
{
  geometry_msgs::Pose2D goalLocation;

       //CNMTargetSearch(currentLocation, goalLocation, cnmCenterLocation);
        if (searchLoop < 0 || searchLoop > 8)
        {
            searchLoop = 0;
            cnmNumRotations++;
        }

        if (searchLoop == 0)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x + searchCounter;
            goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            cnmNumRotations++;
        }
        else if (searchLoop == 1)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x + searchCounter;
            goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            cnmNumRotations++;
        }
        else if (searchLoop == 2)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
            goalLocation.y = cnmCenterLocation.y + searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            cnmNumRotations++;
        }
        else if (searchLoop == 3)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
            goalLocation.y = cnmCenterLocation.y + searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
        }
        else if (searchLoop == 4)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x - searchCounter;
            goalLocation.y = cnmCenterLocation.y + searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
            looping++;

            cnmNumRotations++;
        }
        else if (searchLoop == 5)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x - searchCounter;
            goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
        }
        else if (searchLoop == 6)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x - searchCounter / 2;
            goalLocation.y = cnmCenterLocation.y - searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            cnmNumRotations++;
        }
        else if (searchLoop == 7)
        {
            searchLoop++;
            goalLocation.x = cnmCenterLocation.x + searchCounter / 2;
            goalLocation.y = cnmCenterLocation.y - searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            cnmNumRotations++;
        }
        else if (searchLoop == 8)
        {            
            goalLocation.x = cnmCenterLocation.x + searchCounter;
            goalLocation.y = cnmCenterLocation.y - searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            cnmNumRotations++;

            //if they do not complete at least 4 full rotations
            if(cnmNumRotations >= 4)
            {
                searchCounter = searchCounter + searchDist; //?

                cnmNumRotations = 0;
            }
            else if(cnmNumRotations > 2)
            {
                cnmNumRotations = 0;
            }

            searchLoop = 0;
        }

    return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation)
{

    if (currentLocation.theta <= 45 * M_PI / 180)
    {
        searchLoop = 2;
    }
    else if (currentLocation.theta <= 90 * M_PI / 180)
    {
        searchLoop = 4;
    }
    else if (currentLocation.theta <= 135 * M_PI / 180)
    {
        searchLoop = 5;
    }
    else if (currentLocation.theta <= 180 * M_PI / 180)
    {
        searchLoop = 7;
    }
    else if (currentLocation.theta <= 225 * M_PI / 180)
    {
        searchLoop = 7;
    }
    else if (currentLocation.theta <= 270 * M_PI / 180)
    {
        searchLoop = 0;
    }
    else if (currentLocation.theta <= 310 * M_PI / 180)
    {
        searchLoop = 1;
    }
    else if (currentLocation.theta <= 360 * M_PI / 180)
    {
        searchLoop = 2;
    }

    geometry_msgs::Pose2D newGoalLocation;

    //remainingGoalDist avoids magic numbers by calculating the dist
    // double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

    newGoalLocation = search(currentLocation);

    //this of course assumes random walk continuation. Change for diffrent search methods.
    //newGoalLocation.theta = oldGoalLocation.theta;
    //newGoalLocation.x = currentLocation.x + (0.20 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
    //newGoalLocation.y = currentLocation.y + (0.20 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

    return newGoalLocation;
}

void SearchController::setCenterSeen(bool answer)
{
    cnmCenterSeen = answer;
}

void SearchController::setCenterLocation(geometry_msgs::Pose2D newLocation)
{
    cnmCenterLocation = newLocation;
}

void SearchController::cnmSetRotations(int num)
{
    cnmNumRotations = num;
}

/*
void SearchController::CNMCenterSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation)
{
    if (centerLoop < 0 || centerLoop > 8)
    {
        centerLoop = 8;
    }

    if(centerLoop == 0) //octigonal search pattern
    {
        centerLoop = 8; //
        goalLocation.x = goalLocation.x + centerSearch / 2; //drive east
        goalLocation.y = goalLocation.y - centerSearch;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 1)
    {
        centerLoop-=1; //?
        goalLocation.x = goalLocation.x + centerSearch; //sets drive location to the east branch ammount
        goalLocation.y = goalLocation.y + centerSearch / 2; //and north half that ammount
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 2) //by now we have driven north branch ammount in a location branch east
    {
        centerLoop-=1; //?
        goalLocation.x = goalLocation.x + centerSearch / 2; //drive northwest
        goalLocation.y = goalLocation.y + centerSearch;
        goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 3)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch / 2; //drive west
         goalLocation.y = goalLocation.y + centerSearch;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 4)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch; //drive south west
         goalLocation.y = goalLocation.y + centerSearch / 2;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 5)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch; //drive south
         goalLocation.y = goalLocation.y - centerSearch / 2;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 6)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x - centerSearch / 2; //drive south east
         goalLocation.y = goalLocation.y - centerSearch;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 7)
    {
         centerLoop-=1; //?
         goalLocation.x = goalLocation.x + centerSearch / 2; //drive east
         goalLocation.y = goalLocation.y - centerSearch;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
    else if (centerLoop == 8)
    {
         centerLoop-=1; //?
         centerSearch = centerSearch + searchDist * 2; //select a new radius to search
         goalLocation.x = goalLocation.x + centerSearch;
         goalLocation.y = goalLocation.y - centerSearch / 2;
         goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
    }
}

*/
//Jeff Schlindwein, snippet of code from last years mobility


/*//INITIAL RUN FUNCTION:  Drive forward to see if you find the center, 
void CNMInitial()
{
	first = false;
	rotBool = false;
	goalLocation.theta = currentLocation.theta;
	//drive forward 50cm
	goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
	goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
	circleLocation.x = 0;
	circleLocation.y = 0;
	driftCompensation.x = 0;
	driftCompensation.y = 0;
	//IF so go to a point in the search pattern
	if (currentLocation.theta <= 45 * M_PI / 180)
	{
		searchCounterLoop = 3;
	}
	else if (currentLocation.theta <= 90 * M_PI / 180)
	{
		searchCounterLoop = 2;
	}
	else if (currentLocation.theta <= 135 * M_PI / 180)
	{
		searchCounterLoop = 1;
	}
	else if (currentLocation.theta <= 180 * M_PI / 180)
	{
		searchCounterLoop = 8;
	}
	else if (currentLocation.theta <= 225 * M_PI / 180)
	{
		searchCounterLoop = 7;
	}
	else if (currentLocation.theta <= 270 * M_PI / 180)
	{
		searchCounterLoop = 6;
	}
	else if (currentLocation.theta <= 310 * M_PI / 180)
	{
		searchCounterLoop = 5;
	}
	else if (currentLocation.theta <= 360 * M_PI / 180)
	{
		searchCounterLoop = 4;
	}
}
*/
