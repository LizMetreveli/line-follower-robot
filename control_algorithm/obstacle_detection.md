# Obstacle Detection Algorithm

## Overview
I implemented an obstacle detection behavior layered on top of the line-following control loop.
The algorithm uses ultrasonic sensor readings to detect obstacles and temporarily override line-following behavior.

## Control Logic
The obstacle detection logic is implemented as a conditional branch:
- If no obstacle is detected, the robot continues line following
- If an obstacle is detected, the robot executes a predefined avoidance maneuver

The avoidance sequence consists of:
1. Stopping briefly
2. Turning left for a fixed duration
3. Turning right to realign with the track
4. Resuming line-following behavior

## Challenges and Adjustments
During testing, ultrasonic sensor readings introduced timing delays that interfered with the line-following loop, causing jittery motion.
To mitigate this:
- I reduced the robot’s speed during obstacle detection
- Tuned timing values to balance responsiveness and stability

Given more development time, this logic could be extended to include active line reacquisition after obstacle avoidance.

## Design Trade-Offs
- Simple state-based logic was chosen for reliability
- Fixed-duration maneuvers were favored over complex search behaviors
- Stability was prioritized over speed during obstacle handling

