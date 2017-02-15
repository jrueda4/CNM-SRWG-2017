#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
  geometry_msgs::Pose2D goalLocation;

  //select new heading from Gaussian distribution around current heading
  goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

  //select new position 50 cm from current location
  goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
  goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

  return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}

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

//INITIAL RUN FUNCTION:  Drive forward to see if you find the center, 
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

//SEARCHES for CENETER goes LEFT
void CNMCenterSearch()
{
 
           if (circleCounter == 0 && rotBool == true) //octigonal search pattern
           {
                      circleCounter = 8; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x + branchCounter / 2; //drive east
                      goalLocation.y = circleLocation.y - branchCounter;
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 1 && rotBool == true)
           {
                      circleCounter--; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x + branchCounter; //sets drive location to the east branch ammount
                      goalLocation.y = circleLocation.y + branchCounter / 2; //and north half that ammount
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 2 && rotBool == true) //by now we have driven north branch ammount in a location branch east
           {
                      circleCounter--; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x + branchCounter / 2; //drive northwest
                      goalLocation.y = circleLocation.y + branchCounter;
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 3 && rotBool == true)
           {
                      circleCounter--; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x - branchCounter / 2; //drive west
                      goalLocation.y = circleLocation.y + branchCounter;
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 4 && rotBool == true)
           {
                      circleCounter--; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x - branchCounter; //drive south west
                      goalLocation.y = circleLocation.y + branchCounter / 2;
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 5 && rotBool == true)
           {
                      circleCounter--; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x - branchCounter; //drive south
                      goalLocation.y = circleLocation.y - branchCounter / 2;
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 6 && rotBool == true)
           {
                      circleCounter--; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x - branchCounter / 2; //drive south east
                      goalLocation.y = circleLocation.y - branchCounter;
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 7 && rotBool == true)
           {
                      circleCounter--; //?
                      rotBool = false;
                      goalLocation.x = circleLocation.x + branchCounter / 2; //drive east
                      goalLocation.y = circleLocation.y - branchCounter;
                      goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
           }
           else if (circleCounter == 8)
           {
                      circleCounter--; //?
                      branchCounter = branchCounter + searchDist * 2; //select a new radius to search
                      circleLocation.x = circleLocation.x; +driftCompensation.x;
                      circleLocation.y = circleLocation.y; +driftCompensation.y; //?
 
                      rotBool = true;
           }
 
 
 
}
 
//SEARCHES for TARGETS, goes RIGHT
void CNMTargetSearch()
{
 
           //if no target found but center has been found
           if (targetDetected.data == -1)
           {
                      leaving = true; //bool used after a target has been returned to cause reasearching of some area in order to handle clusters better           
                      if (searchCounterLoop < 0)
                      {
                                 searchCounterLoop = 0; rotBool = true;
                      }
                      if (searchCounterLoop == 0 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x + searchCounter;
                                 goalLocation.y = circleLocation.y - searchCounter / 2;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 1 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x + searchCounter;
                                 goalLocation.y = circleLocation.y + searchCounter / 2;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 2 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x + searchCounter / 2;
                                 goalLocation.y = circleLocation.y + searchCounter;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 3 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x - searchCounter / 2;
                                 goalLocation.y = circleLocation.y + searchCounter;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 4 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x - searchCounter;
                                 goalLocation.y = circleLocation.y + searchCounter / 2;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 5 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x - searchCounter;
                                 goalLocation.y = circleLocation.y - searchCounter / 2;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 6 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x - searchCounter / 2;
                                 goalLocation.y = circleLocation.y - searchCounter;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 7 && rotBool == true)
                      {
                                 searchCounterLoop++;
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x + searchCounter / 2;
                                 goalLocation.y = circleLocation.y - searchCounter;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      else if (searchCounterLoop == 8 && rotBool == true)
                      {
                                 searchCounter = searchCounter + searchDist; //?
                                 searchCounterLoop = 0;
                                 circleLocation.y = circleLocation.y; +driftCompensation.y;
                                 circleLocation.x = circleLocation.x; +driftCompensation.x; //?
 
                                 rotBool = false;
                                 goalLocation.x = circleLocation.x + searchCounter;
                                 goalLocation.y = circleLocation.y - searchCounter / 2;
                                 goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
                                 looping++;
                      }
                      if (looping > 7)
                      {
                                 looping = 0;
                                 driftCounter++;
                      }
 
           }//end else if for no targets found
 
 
}*/
