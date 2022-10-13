# Robot Localization using a Particle Filter
Computational Robotics, Fall 2022  
Lilo Heinrich and Tigey Jewell-Alibhai  

### Goal
Create a particle filter which uses a robot's lidar scan data to determine an estimate of the robot's pose and location on a given map. 

### Method 
We tested our particle filter using two methods: a gazebo simulation of a Neato robot driving around the Gauntlet world controlled using keyboard input, and bag files of the Turtlebot robot driving around the Academic Center 1st floor. We were provided maps for both settings.

To initialize the particle cloud, we used `np.random.normal()` to create normal (Gaussian) distributions of x, y, and theta centered around the initial pose estimate that we provide in rviz. We set the scale factor for the x- and y- distributions to be 0.1, and the scale factor of the theta distribution as 0.005. These values were found experimentally. The scale factor for a normal distribution is equal to its standard deviation. Initially, we had a higher scale value for theta but the particles' headings were not well related to the robot's actual heading, which caused a problem when trying to estimate and visualize the robot's pose from the particle cloud. With a theta scale of 0.005 the particle headings stay near parallel, which worked well for creating a robot pose estimate in our two test situations.

The particle weights are updated by overlaying the scan data around each particle's pose, then for each lidar data point in the scan finding the distance to the closest obstacle in the occupancy field of the map. Summing up this distance between each scan point and the nearest obstacle gives an approximation of how well that particle's pose is able to match up the lidar scan to the map. Each particle's weight is assigned to be 1 divided by the mean distance squared, making it so that a lower average distance causes a higher particle weight. 

The particles are resampled according to their weights, where each particle's weight (after normalizing all the particle weights) defines the probability of that particle being selected. This method prunes away the particles which have a lower weight and therefore a worse fit, keeping more instances of those particles which had a better fit. The sampled particles are then run through the same normal distribution code for initializing the particle cloud, but this time centered on each sampled particle's position. 

Lastly, the robot's pose is computed from the particle cloud by taking the average x, y, and theta value of the particles. We also tried another method  where we computed the robot's pose as the most likely pose (the mode of the distribution), but the particle filter's behaviour was not as good becuase this tended to create bigger changes in pose from one calculation to the next, so the robot pose estimate jumped around more. The most likely pose also did not set the robot's pose as accurately, so we found the average to be more robust. 

We faced a challenge when trying to update and visualize the robot's pose in rviz, where the robot and the scan data were not overlaying correctly over the map, even though the particle cloud was correctly following the robot's position on the map. It seemed that the frame of the robot and scan data was rotating around the origin (0,0) rather than around the center of the particle cloud. It also seemed there was a problem with the x- and y- scaling factor. We tried to debug this for over an hour and thought it was a problem with our robot pose code, but it turned out to be caused by a bug which Paul fixed that we hadn't synced. It was frustrating because we thought we had done something to cause this error and that we could fix it in the particle filter coe, but it turned out to be a bug in the helper function that does the map to odom frame transformation.

If we had more time, I'd like to test our particle filter on a real Neato robot driving around the Academic Center, or try to make our particle filter work even without an initial pose estimate (although since the Academic Center has many self-similar areas, it may be difficult).


    add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

    What was the goal of your project?
    How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
    Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
   
