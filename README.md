# particle_filter_localization_project

## Implementation Plan

1. The names of your team members
Jonah Kaye, Yoon Jeong

1. How you will initialize your particle cloud (initialize_particle_cloud)?
	
    **Implementation:** We will generate uniformly random points and put it into an array called `self.particle_cloud` using the Python "random" library. We will use a for loop to do this. We will use as many random points as possible without compromising on efficiency. 

    **Test:** We will do an eye check by printing the variable (using "echo") to make sure that it appears random and is within the bounds of what we have set. We can print the length of the array to ensure that it has the number of initial points we expect.
1. How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
	
    **Implementation:** We will subscribe to the odometer rostopic and add the distance travelled to the particles in the particle cloud, and will also add Gaussian noise by using the Python random library. We will use a for loop to iterate through the particle cloud.
	
    **Test:** We will instruct our robot to go a certain distance, and as above, echo our particle cloud array variable and do an eyeball check to see that the points have been updated.
1. How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?

    **Implementation:** We will subscribe to the scan rostopic and convert the ms.ranges into the z_t(m) value. Then, we will use the measurement weight equation from class, taking the sum of 1/abs(Z_t(i) - Z_t(m)) and then we will put it into an ordered weight list that is stored in the class.

    **Test:** We will take an example, having instructed our robot to go only a certain distance, and then check by hand that the robot calculations are correct. 

1. How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
	
    **Implementation:** To normalize the weights of the particle, we will sum up all the particle weights and then take the inverse, and then multiply each weight by that inverse. We will use a for loop to iterate through the particle weights. There is a python function in the "random" library called `choices` that will allow us to select weights to resample with the appropriate probabilities.
	
    **Test:** To test that normalization worked properly, we can check to see that the list sums up to 1. To see if the resampling is working properly we can see if the visualization we create is converging upon certain peaks and removing a good number of particles per time step.
1. How you will update the estimated pose of the robot (update_estimated_robot_pose)?

    **Implementation:** We will take the average of all the points that have been resampled in order to find the estimated pose.

    **Test:** To test this, we will put it into rviz and do an eyeball check to see that the pose approximates the cloud of particles.
1. How you will incorporate noise into your particle filter localization?
	
    **Implementation:** We will be incorporating noise by using the normal function available from the Python random library.
	
    **Test:** To test this, we will see if rviz produces a cloud of particles rather than slowly converging on individual points.

1. A brief timeline sketching out when you would like to have accomplished each of the components listed above.

**By this weekend:** We want to be fully finished with `initialize_particle_cloud` (Yoon) and `update_particles_with_motion_model` (Jonah). We want to have started `update_particle_weights_with_measurement_model` (Yoon) and `normalize_particles` (Jonah) and `resample_particles` (Jonah).
**By next Wednesday:** We want to be finished with writing all the coding aspects and be working on debugging.
This timeline may change as we work on it.

