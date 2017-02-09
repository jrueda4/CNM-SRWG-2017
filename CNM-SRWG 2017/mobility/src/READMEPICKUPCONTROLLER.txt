PICKUPCONTROLLER

--------2/8/2017--------
EDITED BY:  Jeff Schlindwein, Paul Ward

CHANGES:
- Altered  float targetDist IN PickUpResult PickUpController::pickUpSelectedTarget(bool blockBlock)
  + changed from .25 to .20
  + altered because robots were not driving close enough to pick up blocks in simulation; may need to change on physical 
      rovers
