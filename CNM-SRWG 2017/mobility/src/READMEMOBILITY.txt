MOBILITY

--------2/8/2017--------
EDITED BY:  Jeff Schlindwein, Steve Lindsey, Kaily Young, Juan Rueda, Paul Ward

- UPDATED Timer Information
  + added new Timer for initial beginning behavior (turn 180 degrees and drive to get out of the center) 
    (LINES: 197, 198, 274, 500, 848, 1091)
  + added publisher for when we see the nest for the first time
    (LINE 492)
  + working on code that offsets center location from where robot last see's tag 256
    - currently works when robot is spinning 180, but gets messy after that.
      + loop gets noticeably disorganized
- TODO:
  + add better behavior for navigating after seeing center
  + work on behavior for spreading out robots
  + start delving into pickup and drop off behavior
  
- ISSUES:
  + When running rovers only, the SIMULATION will acknowledge the existance of the new initial TIMER
  + When running rovers AND targets, the SIMULATION REFUSES to acknowledge there is a TIMER OR a BOOLEAN
    trigger for the init behavior.
      - must test on REAL ROVERS to see actual behavior

--------2/19/2017--------
EDITED BY:  Jeff Schlindwein

- changed obstacle detection series to run off a timer as opposed to running off of counter  
  + (LINES:  191, 262 , 600, 627, 1033)
  + increases accuracy and more easily understood time increments (set at line 189)
  + allows us to follow this timer model for other behavior
- added publisher messages for when avoiding obstacles.  Watch the log box in the rover GUI for the updates

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

  TODO:
  - REPLACE the current counter with a timer that is set for more precise behavior

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
