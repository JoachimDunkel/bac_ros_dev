## Original package:
https://github.com/iralabdisco/ira_laser_tools

## Changes I made:

* Got rid of laser_scan_virtualizer
* Got rid of dynamic reconfiguring
* Got rid of publishin on a combined laser / only point-cloud is needed
* Cleaned up CMake and Build-depencies to not give of warnings anymore.
* Changed range default params to -pi / pi. (360°)
  
## TODO 
* Change scan_time range_min and range_max to the correct values of the evo-robot
