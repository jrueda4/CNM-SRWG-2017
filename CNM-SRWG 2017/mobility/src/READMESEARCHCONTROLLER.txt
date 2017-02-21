SEARCHCONTROLLER.CPP

--------2/20/2017--------
EDITED BY:  Jeff Schlindwein, Steve Lindsey, Kaily Young, Juan Rueda, Paul Ward
- Incorporated <angles/angles.h> to header file
  + gives us access to radians to degrees conversion functions
    - used new conversion functions to simplify interruptedSearch
- Adjusted search pattern distances (randomly generated numbers that dictate how far we start away from the center)
  + values are (.7, 2.01) from (.9, 1.56)
  + this gives us much greater variety for robots to choose a search line that is different than another robots
  + searchDistance may be increased (currently set to .2 which is the width of a robot)

--------2/18/2017--------
EDITED BY:  Jeff Schlindwein

- finished implementing a full octagonal search pattern along with the continueInterruptedSearch function
  + Search is the main code for deriving the next point to acheive based on the centers central point
  + continueInteruptedSearch finds which angle it is currently facing and then makes a presumption of where it should continue the search pattern based on this
    - Numbers that correlate to eachother are as follows:
      + 0 => 4
      + 1 => 5
      + 2 => 6
      + 3 => 7
      
      + 8 in the loop is only there to increment the search pattern, so thus is not treated as a real point

- in order to use this appropriately, please refer to the mobility.cpp readme. The calls are made from within that file and the readme will be able to explain how they work within it.

- at the bottom of the file, commented out, is the full snippet of code from last years mobility changes (in terms of the octagonal search).


----------2/16/2017----------
EDITED BY:  Jeff Schlindwein

- Implemented original Octogon based search algorithm
- working on making it pretty
