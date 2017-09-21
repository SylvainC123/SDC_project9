# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model Predictive Control

### The project 
The project consists in driving a car in a simulator based on an MPC model.
The simulator input data (and model output) are: 


- steering and throttle data

The model input (and simulator output) are:

 - the path guideline
 - the vehicle kinematic parameters
	 - position x, y, orientation
	 - speed v, steering angle 

### The Model

Here are the main steps of the program (main.cpp file):

- the "path guideline" is expressed in the vehicle local coordinates
then approximated by a parabola (second degree polynomial).
- the vehicle characteristics after a latency period (actuator time) are computed based on current vehicle characteristics.
- from this latency position, a trajectory is computed by minimizing a cost function defined relative to the "path guideline" .
- the first step of this trajectory is transferred to the simulator as a throttle and a steering value.

The vehicle model is hidden into the optimization parameters (MCP.cpp file) :

- It is a kinematic model, the vehicle dynamic are not considered at this stage (inertia forces, tyre behaviour, steering-suspension model,...).


- in the computation of the successive steps of the trajectory


	- x ​t+1 ​= x​t +v​t ∗ cos(ψ​t)∗dt


	- y ​t+1​​ = y​t​​ +v​t​​ ∗ sin(ψ​t)∗dt


	- ψ ​t+1​​ = ψ​t​​ +​L​f ∗ ​v​t ∗ δ ∗ dt


	- v ​t+1​​ = v​t + a ∗dt


	- cte ​t+1 = cte ​t + v​t ∗sin(eψ​ t) ∗ dt


	- eψ ​t+1​​ =eψ ​t ​+ ​L​f ∗ ​v​t ∗ δ​t ∗dt

- in the optimisation constraints
	- limiting steering angle to [-25,25] deg
	- limiting acceleration to [-1,1]




- in the definition of the cost parameters for the optimisation as a function of :
	- cross track error
	- orientation error
	- gap to target speed
	- throttle gap
	- steering gap
	- throttle continuity
	- steering continuity

###hyperparameters
####timeframe and steps

`size_t N = 50;
double dt = 0.1;`

I choose to have a time horizon of ~5s which at 50 mph correspond to ~111m which is the order of magnitude of half the perimeter of the steeper of the circuit bend.

I choose a dt of 0.1s the same order of magnitude as the latency time, leading to 100 iteration, which is steel reasonable for the computing power of my PC.

### cost parameters
I selected the following parameters:

    /// init target speed
    #define ref_v 		50

    /// hyperparameters for the cost function
    #define w_cte 		100		// cross track error
    #define w_epsi 		1		// direction error
    #define w_dv 		100		// gap to target speed
    #define w_a 		1		// throtle gap
    #define w_delta 	100		// steering gap
    #define w_d_a 		0		// throtle continuity factor
    #define w_d_delta 	1000000	// steering continuity factor

The tuning of the "cost" hyper-parameters was done manually, the process was as follow :

- 1. Select a very large weight for the "steering continuity" to provide a smooth trajectory.
- 2. Select moderate weight for the "target speed parameter".
- 3. Increase the weight of the cte to keep the optimisation close to the guide line.

The tuning of the parameter is a try and error process, with multiple possible choices.

##Result
The car run an ~ 40mph in average, higher in straight portions and slows down in the turns as expected.

It is still a bit wobbly but do not seem to go off track.

Some further optimisation can be performed those parameters seems to do the job.
