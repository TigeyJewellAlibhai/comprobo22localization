# Robot Localization using a Particle Filter
Computational Robotics, Fall 2022 

Lilo Heinrich and Tigey Jewell-Alibhai

### Goal
Our goal was to create a particle filter which uses a robot's lidar scan data to determine an estimate of the robot's pose and location on a given map. 

### Method 
We tested our particle filter using two methods: a gazebo simulation of a Neato robot driving around the Gauntlet world which we could control using keyboard input, and bag files of the Turtlebot robot driving around the Academic Center 1st floor. We were provided maps for both settings.

To initialize the particle cloud, we used `np.random.normal()` to create normal (Gaussian) distributions of x, y, and theta centered around the initial pose estimate that we provide in rviz. We set the scale factor for the x- and y- distributions to be 0.1, and the scale factor of the theta distribution as 0.005. These values were found experimentally. The scale factor for a normal distribution is equal to its standard deviation. Initially, we had a much higher scale value for theta but the particles' headings were not well related to the robot's actual heading, which caused a problem when trying to estimate and visualize the robot's pose from the particle cloud. With a theta scale of 0.005 the particle headings stay near parallel, which works well for creating a robot pose estimate in our two test situations.





    add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

    What was the goal of your project?
    How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
    Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
   
