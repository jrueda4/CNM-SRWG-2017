MOBILITY

--------2/8/2017--------
EDITED BY:  Jeff Schlindwein, Steve Lindsey, Kaily Young, Juan Rueda, David Kirby, Rudy Garcia, Kristin Markle, Paul Ward

CHANGES:
- Created a section for CNM Code to go
- Moved speed variable to CNM Code Section for easy finding
- Created Function void CNMPickUpReturn()
  + Code for the STATE_MACHINE_PICKUP in MOBILITYSTATEMACHINE
- Moved PickUpController to cnmPickupReturn()
- Created bool variable cnmPickupReturn
  + Default instantiated to FALSE
  + used to keep track on if the code needs to send a return

TODO:
  - Move other MOBILITYSTATEMACHINE code in switch statement to seperate functions
