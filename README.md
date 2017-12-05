# drake-new-code

# Footstep Planning Code for Drake #
Python code written for the new Drake distro.
Before using, `source setup.sh`.

## Todo ##
[ ] Make the reachable regions from footsteps into general convex regions
[ ] Add a nominal and reachable distinction (so it is much costlier to step outside of some scaled-down version of the reachable region)
[ ] Add an option to make the reachable region into multiple convex regions
[ ] Use LCM (eventually, use it to build into the old MATLAB planner to allow for simulation of created footstep plans)

## [convex-region-optimization](convex-region-optimization) ##

### [2D-qhull.py](convex-region-optimization/2D-qhull.py) ###
Can solve for the closest point within a number of 2D regions to a goal point (run with -s to save the outputted image).
![2D-qhull.py output](/convex-region-optimization/images/2D-6UK6PM.png?raw=true "2D-qhull.py Output")

### [3D-qhull.py](convex-region-optimization/3D-qhull.py) ###
Can solve for the closest point within a number of 3D regions to a goal point (run with -s to save the outputted image).
![3D-qhull.py output](/convex-region-optimization/images/3D-W3TGPS.png?raw=true "3D-qhull.py Output")
