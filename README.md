# Modified DFRobot ECPH library
Description:

DFRobot manufacturer provides 2 separate libraries for their EC probe and pH probe.
Blocking issue is discovered when applying both libraires together in the same coding program.
To resolve the issue, merging these 2 libraries into one single library is performed.
The code is also further modified by adding command handlings for the "ECPHUP" and "ECPHDOWN".
The added commands are used for nutrient regulation control.

## Declaration
The 2 orignal libraries are taken from https://github.com/greenponik/DFRobot_ESP_EC_BY_GREENPONIK and https://github.com/GreenPonik/DFRobot_ESP_PH_BY_GREENPONIK as references to produce the Modified DFRobot ECPH library.
