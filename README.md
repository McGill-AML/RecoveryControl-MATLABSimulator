# Matlab-Simulator
Contact **Fiona Chui** for questions on quadrotor dynamics (including contact), non-recovery controllers, collision detection, and collision characterization using Fuzzy Logic. Contact **Gareth Dicker** for questions on recovery controller.

MATLAB simulator that captures contact dynamics during collision between a propeller-protected quadrotor and a wall. The collision is identified and characterized, then a Recovery Controller stabilizes the vehicle to a safe distance away from the wall.

 - Start standard simulator with `startsim.m`
 - Initial collision conditions can be prescribed inside `startsim.m`
 - `startsim_matchexp.m` is for creating simulations to mimic collisions using Spiri platform (set I)
 - `startsim_matchexpoffboard.m` is for creating simulations to mimic collisions using Navi platform (set VII)
 - `startsim_trajectory.m` is for prescribing a vehicle trajectory. Useful for non-collision simulations (make sure to locate `ImpactParams.wallLoc` far away from the trajectory waypoints.
 - `sim_Batch.m` runs `startsim.m` recursively (once you have turned it into a function by uncommenting the first line), to simulate collisions for an array of specified initial conditions. sim_MonteCarlo.m is recommended over this option.
 - `sim_MonteCarlo.m` runs `startsim.m` recursively (once you have turned it into a function by uncommenting the first line), to simulate collisions for an array of randomized initial conditions within specified ranges.
