# Robot Localization using a Particle Filter
Computational Robotics, Fall 2022  
Lilo Heinrich and Tigey Jewell-Alibhai  

### Goal
Create a particle filter which uses a robot's lidar scan data to determine an estimate of the robot's pose and location on a given map. 

### Results
We tested our particle filter using two methods: a gazebo simulation of a Neato robot driving around the Gauntlet world controlled using keyboard input, and bag files of the Turtlebot robot driving around the Academic Center 1st floor. We were provided maps for both.

In the end our particle filter tracks the robot's position well in both test scenarios and we are satisfied with the result. The point cloud, although it is fairly spread out, stays centered on the robot's actual position. 
- INSERT GIFS of PF in action

### Method 
To initialize the particle cloud, we used `np.random.normal()` to create normal (Gaussian) distributions of x, y, and theta centered around the initial pose estimate that we provide in rviz. We set the scale factor (aka the standard deviation) for the `x and y distributions as 0.1`, and the scale factor of the `theta distribution as 0.005`. These values were found experimentally. <!-- The scale factor for a normal distribution is equal to its standard deviation.  --> Initially we had a higher scale value for theta but the particles' headings were not well related to the robot's actual heading, which caused a problem when trying to estimate and visualize the robot's pose from the particle cloud. With a theta scale of 0.005 the particle headings stay near parallel, which worked well for creating a robot pose estimate in our two test situations.

The particle weights are updated by overlaying the scan data around each particle's pose, then for each lidar data point in the scan finding the distance to the closest obstacle in the occupancy field of the map. Summing up this distance between each scan point and the nearest obstacle gives an approximation of how well that particle's pose is able to match up the lidar scan to the map. Each particle's `weight is 1 divided by the mean distance squared`, making it so that a lower average distance causes a higher particle weight. 

The particles are resampled according to their weights, where each particle's weight (after normalizing) defines the probability of that particle being selected. This prunes away the particles which have a lower weight and therefore a worse fit, keeping more instances of those particles which had a better fit. The sampled particles are then run through the same normal distribution code for initializing the particle cloud, but this time centered on each sampled particle's position. 

Lastly, the robot's pose is computed from the particle cloud by taking the `average x, y, and theta value` of the particles. We also tried setting the robot's pose as the most likely pose (the mode of the distribution), but the particle filter's behaviour was not as good because this tends to create bigger changes in pose from one calculation to the next, so the robot pose estimate jumped around more. The most likely pose also didn't set the robot's pose as accurately, so we found the average to be more robust. 
- INSERT GIFs of average vs. most likely


### Challenges
We faced a challenge when trying to update and visualize the robot's pose in rviz, where the robot and the scan data were not overlaying correctly over the map, even though the particle cloud was correctly following the robot's position on the map. The frame of the robot and scan data was rotating around the origin (0,0) rather than around the center of the particle cloud and there was a problem with the x- and y- scaling factor. We tried to debug this for over an hour and thought it was a problem with our robot pose code, but it turned out to be a bug which Paul fixed that we hadn't synced. It was frustrating because we thought we had caused this error and that we could fix it in the particle filter code, but it turned out to be a bug in the helper function for the map to odom frame transformation.

### Reflection
Lilo:  
The pieces of our project didn't come together until the rviz visualization error was fixed, so I learned that visualization is extremely important. Although rviz can be frustrating to learn to use and to configure correctly, it is an important and useful tool when working with large and constantly changing data such as lidar. I'm wondering what visualization tools we can use in the next project, perhaps annotating image data. 

This project is one of my first projects working with mainly simulations and bag files, and I am learning to appreciate being able to run simulations before running on a real robot. But I still prefer when what I am working on connects back to a real, physical system rather than just a simulator. Although simulators can allow us to focus more on the algorithms than integration, I also enjoy integration and the challenge of making something work in the real-world as well as having a demo to show. That is why I am hoping to work with a real drone in the Computer Vision project.

If we had more time, I'd like to test our particle filter on a real Neato robot driving around the Academic Center. Another thing I'd have liked to do more of is working on tuning our distribution scaling, particle weights, and pose computation method to optimize the performance of the Particle Filter. 



### Prompts
    Add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).
    *What was the goal of your project?
    *How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
    *Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    *What if any challenges did you face along the way?
    *What would you do to improve your project if you had more time?
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
