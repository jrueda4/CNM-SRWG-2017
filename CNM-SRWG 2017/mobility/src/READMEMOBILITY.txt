MOBILITY

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
