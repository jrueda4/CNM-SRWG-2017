MOBILITY

--------2/18/2017--------
EDITED BY:  Jeff Schlindwein

- Moved beginning initialization into a CNM INIT file.  This is not a permenant place, but this can be manipulated to add beginning behavior before conducting search pattern

- Full changes to Obstacle Handling goes as follows:
  + Under void ObsacleHandler() (LINE 545)
    - First change is a bool called cnmAvoidObstacle
      + initially set to false, only triggered true if an obstacle is detected past its 45 counter limit (read below for more details).
      + if this is triggered to be true, find a point 90 degrees to the left and drive to it
      + Once the sensors report no obstacle, trigger cnmAvoidObstacle back to false
    - Second change is in the main if statement (if !targetdetected || !collected)
      + if we see an obstacle wait for a counter to increment to 45 (based on cycle rate of the program)
      + once it increments to 45 we have been sitting for ~20-30 seconds
      + change behavior by triggering cnmAvoidObstacle bool
      
- Full changes to how searchController works as follows:
  + if we are driving from point a to point b, call searchController.search to get the next point
  + if we come across a target, obstacle, or anything that changes our previous point, call interrutedSearch function
    + check searhController.cpp readme for interruptedSearch and Search function information.
    
- if we see the center location and are not carrying anything
  + back up until it is no longer in view
  + call interruptedsearch function.

--------2/16/2017--------
EDITED BY:  Jeff Schlindwein

- Reworked obstacle detection
  + added boolean triggers and re-implemented old obstacle design
  + made the robot treat all obstacles the same (turn left)

--------2/15/2017--------
EDITED BY:  Jeff Schlindwein

- Organized all variable and class object instantiations.
  + #defines
  + Class Objects
  + standard variables
  + Functions
  + Publishers/Subscribers/Timers
  + Callback Handlers
  + CNM Code
- Moved rest of the main MOBILITY STATE MACHINE into dedicated functions
  + functions are under CNM Code Follows:
- Finished cleaning curly braces for readability


--------2/8/2017--------
EDITED BY:  Jeff Schlindwein, Steve Lindsey, Kaily Young, Juan Rueda, David Kirby, Rudy Garcia, Kristin Markle, Paul Ward

CHANGES:
- Created a section for CNM Code to go
  + line 163
- Moved speed variable to CNM Code Section to find more easily
  + float searchVelocity = 0.4 // originally .2
- Created Function void CNMPickUpReturn()
  + line 442
  + Code for the STATE_MACHINE_PICKUP in MOBILITYSTATEMACHINE
- Moved PickUpController to cnmPickupReturn()
  + line 743
- Created bool variable cnmPickupReturn
  + line 170
  + Default instantiated to FALSE
  + used to keep track of if the code needs to send a return statement

TODO:
  - Move other MOBILITYSTATEMACHINE code in switch statement to seperate functions
