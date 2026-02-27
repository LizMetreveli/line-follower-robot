## Wiring Diagram (Reference)
I designed and iterated on the robot’s electrical system throughout development. 
While the initial circuit diagram includes components that were later replaced, the overall system architecture and signal routing remained consistent.

### Power Distribution and Servo Control
During early testing, the servo motor could not be reliably powered from the microcontroller’s 5V output, despite being nominally compatible. 
To resolve this issue, I redesigned the power distribution by adding a dedicated 6V supply derived from the battery pack used for the DC motors. 
This modification provided sufficient torque and stable operation for the steering servo.

I also iteratively adjusted the servo wheel mounting height to improve mechanical alignment and steering response.

### Motor Driver Integration
The motor drivers shown in the initial diagram were later replaced with a custom motor driver circuit.
I updated the wiring and signal routing accordingly while maintaining compatibility with the existing control logic.
This change improved reliability and allowed better integration with the rest of the electrical system.

### Sensor Circuit Redesign
The original Pololu IR sensors did not provide reliable readings during testing.
As a result, I replaced them with a custom IR sensor array based on a discrete circuit design, including current-limiting resistors and appropriate signal conditioning.
The new sensor circuit significantly improved line-detection consistency and overall system stability.

### Documentation Note
The wiring diagram included in this repository was created by me to support development and debugging.
It is provided for reference purposes only. Exact components used were different.
