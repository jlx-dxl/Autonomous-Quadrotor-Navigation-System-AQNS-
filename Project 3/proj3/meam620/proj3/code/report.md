# Proj3 Report Linxuan Jia

## 1. The difference between Proj 1-3 and Proj 3

The shift from Project 1 to Project 3 marks a significant change in the source and reliability of state information. In the earlier projects, the true state of the robot—its position and velocity—was directly provided, allowing control algorithms and path planning to operate based on completely accurate information. In this scenario, the error mean is zero and the variance is also zero, reflecting the absence of uncertainty in the state information.

However, as we progress to Project 3, the state must be estimated using visual inertial odometry (VIO) techniques. In this context, state estimation typically employs an Extended Kalman Filter (EKF) to mitigate sensor noise and compensate for incomplete data. Although EKF effectively reduces these errors, the estimated state still retains a degree of error and uncertainty.

Given these changes, the transition to Project 3 necessitates critical adjustments in the control and planning systems:

- Enhanced fault tolerance: The control strategy needs to exhibit robustness against estimation errors, maintaining effective control performance even within certain error margins.
- Expanded safety boundaries: When planning paths, the uncertainty inherent in state estimation necessitates larger safety margins to preemptively mitigate the risk of potential collisions that could arise from inaccuracies in state estimation.

Hence, I mainly changed two things compared with Project 1: margin and PID parameters, more specificly:

- increase obstacle margin
- increase k_p and k_R
- increase k_d and k_w

## 2. Why did you need to make those changes?

- Why increasing obstacle margin?

Even with advanced estimation techniques like the Extended Kalman Filter (EKF), the state estimates of position and velocity are not perfectly accurate and come with inherent uncertainties. These errors can mean that the robot's perceived position relative to an obstacle is not entirely accurate. By increasing the obstacle margin, you create a buffer zone that compensates for these inaccuracies, reducing the risk of the robot inadvertently colliding with an obstacle due to a misestimated state. And larger margins can lead to smoother trajectories as the planning algorithms avoid tight maneuvers that might be necessary with smaller margins. This can reduce the wear and tear on the robot's mechanical systems and can also lead to more efficient energy use.

- Why increasing k_p and k_R?

In the PID controllers used for both the position and attitude loops of my project, an increase in the proportional gains has been implemented. This adjustment was based on observations from Project 1's code where it was noted that neither the position controller nor the attitude controller reached overshooting, suggesting that there was room for improvement. By enhancing the proportional control's weights, the UAV's responsiveness to deviations from its desired trajectory and orientation has been significantly improved.

The proportional component of a PID controller is crucial for minimizing the error between the current state and the target state. Increasing the proportional gain results in a more assertive response to any discrepancies, thereby boosting the UAV's capability to accurately track the desired position and attitude. However, care was taken to ensure that the increase in gain did not compromise stability or introduce oscillations.

- Why increasing k_d and k_w?

Differential control, integral to PID systems, focuses on the rate of change of the error, providing a damping force that counteracts rapid movements or fluctuations. By increasing k_d and k_w , the system gains a heightened ability to smooth out the effects of sudden shocks or spikes in measurement errors from the VIO system. This adjustment is crucial, particularly in dynamic environments where abrupt changes in sensor readings can lead to equally abrupt and potentially destabilizing changes in control inputs.

Enhancing the differential component helps to dampen these rapid fluctuations, leading to smoother and more stable responses. It prevents the controllers from reacting too aggressively to momentary errors in position and attitude estimates, which can exacerbate the system's overall instability. By doing so, it significantly improves the system's tolerance to inaccuracies and transient disturbances, ensuring more reliable and stable UAV operations. This strategic tuning of the PID controllers not only addresses the immediate challenges posed by noisy data but also bolsters the overall resilience and effectiveness of the control system in maintaining desired trajectories and orientations under uncertain conditions.

- Results:

In Proj 1-3 the total time for all the six maps is 101.7s;

However in Proj 3 the total time is 83.3s;

## 3. What else I want to discuss?


If more time were available, I would consider dynamically adjusting the control parameters based on the state of surrounding obstacles and the UAV's own state. This approach is motivated by the realization that different environments and flight dynamics—such as obstacle-dense versus sparse areas, high-speed versus low-speed conditions, and straight versus continuous turning flights—can influence the static error and variance characteristics of the UAV's position and attitude estimation. These varying conditions may necessitate distinct control parameters to achieve optimal control effectiveness. Dynamically tuning the control parameters in response to the UAV's immediate environment and motion state could greatly enhance precision and stability, adapting in real-time to the specific challenges presented by the flight scenario. This adaptive control strategy would ensure that the UAV maintains high performance and safety under diverse operational conditions.
