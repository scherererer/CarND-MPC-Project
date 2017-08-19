# CarND Model-Predictive Control (MPC)
Michael P. Scherer

---

## The Model

_Student describes their model in detail. This includes the state, actuators and
update equations._

A kinematic model was chosen for its simplicity. Because the MPC is working in
vehicle space, the initial state's {x, y, psi} values are all equal to 0
(vehicle is the center of the world). The target waypoints are transformed into
vehicle space and the values for cte and epsi are calculated from a 3rd order
polynomial fitted to the transformed waypoints.

### Model Update Equations:

	x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
	v_[t+1] = v[t] + a[t] * dt
	cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

The actuators themselves are found using the ipot solver. The solver is
influenced by a cost function that takes the following into account:

- Cross track error
- Orientation error
- Desired speed
- Actuator over-use
- Change in actuator values

See `FG_eval::operator()` in `MPC.cpp` for more details.


## Timestep Length and Elapsed Duration (N & dt)

_Student discusses the reasoning behind the chosen N (timestep length) and dt
(elapsed duration between timesteps) values. Additionally the student details
the previous values tried._

I chose a value of 10 for N. I started with 20 but it seems to work just as well
with less so we might as well reduce the computational cost.

I wanted dt to be approximately equal to the latency (100ms) because I'm
sampling the second time step's actuator values to handle the actuator latency.
I tried setting it to exactly 100ms but found through experimentation that 150ms
was smoother.



## Polynomial Fitting and MPC Preprocessing
	
_A polynomial is fitted to waypoints._

_If the student preprocesses waypoints, the vehicle state, and/or actuators prior
to the MPC procedure it is described._

The waypoints were first converted into vehicle space before fitting with a 3rd order
polynomial. This was done in order to simplify the logic for the MPC calculations,
in particular the calculation of the Cross-Track Error (cte) and the orientation
error (epsi). Additionally, it meant that the output of the MPC was in vehicle
space, which was the required interface for the visualization.

This does, however, incur some errors. The reason for this is that the path is
actually in the space of the state of the vehicle in its initial condition. And
so technically, when comparing the path to a future state of the vehicle it is
not as accurate. Fotunately this proved to be negligible enough that it didn't
matter.


## Model Predictive Control with Latency

_The student implements Model Predictive Control that handles a 100 millisecond
latency. Student provides details on how they deal with latency._

For both steering and throttle, I skipped ahead to the second command. This
handles the latency of the actuators because the dt value is approximately equal
to the latency of the actuators. In other words, the second command is for where
we want the actuator to be at time `t+dt` in the future. And since
`t+dt ~= t+latency`, we can just take the second command as the one that we will
want to execute by the time the actuator recieves it.

In practice, `dt` was chosen to be slightly larger than the latency because it
was found to be more stable.
