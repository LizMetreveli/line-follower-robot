# Line-Following Algorithm

## Overview
I implemented a line-following control algorithm using IR sensor feedback and a closed-loop control strategy.
Unlike common differential-speed approaches, the robot uses a front-mounted servo for steering while keeping the rear wheel speeds initially constant, mimicking a car-like steering model.

## Initial Control Approach
The initial implementation applied a PID-style control structure to compute steering correction based on sensor error.
Sensor readings were weighted to estimate the robot’s position relative to the line, and the resulting error was used to adjust the servo angle.

In this version:
- The same error term was used for proportional, integral, and derivative calculations
- Only the servo angle was adjusted
- Rear wheel speeds remained constant

While this approach allowed the robot to complete the course, it struggled with sharp turns and exhibited overshoot due to chassis length and limited steering range.

## Optimization and Final Algorithm
To improve stability and responsiveness, I simplified the control strategy:
- Removed the integral and derivative terms to reduce overshoot
- Retained proportional control for steering angle adjustment
- Introduced proportional speed adjustment for the rear wheels based on error magnitude

This change allowed the robot to slow down during sharp turns while maintaining smoother steering behavior.
The simplified controller significantly reduced oscillation and improved overall performance.

## Key Design Decisions
- Simplifying control logic improved real-time stability
- Speed modulation was more effective than aggressive steering corrections
- A proportional-only approach was sufficient for the system constraints

## Control Flow Diagram
![Line Following Flow](diagrams/line_following_flow.png)

