# Autonomous Boe-Bot Navigation with Obstacle Avoidance
An autonomous mobile robot developed using a Parallax Boe-Bot, Arduino/C++, a 5-channel line sensor, and ultrasonic sensing. 

The robot autonomously:
-> selects a target destination based on its initial sensor position;
-> exits the starting branch and enters the main navigation map;
-> follows a predefined line track;
-> detects and counts route events;
-> identifies the selected destination junction;
-> detects and avoids obstacles;
-> reacquire the line after obstacle avoidance; and
-> enters and stops at the selected destination.
The completed system was successfully demonstrated during the final project showcase.

# Demo

# Project Objectives
The objective of this project was to develop an autonomous navigation system capable of operating on a predefined track containing:
-> multiple destination branches;
-> rounded corners;
-> starting branches;
-> route junctions; and
-> unexpected obstacles.
Instead of relying on a completely hard-coded movement sequence, the robot uses sensor feedback and a Finite State Machine (FSM) to determine its actions during navigation.

# Key Features
-> 5-channel sensor-based line following
-> Dynamic destination selection 
-> Autonomous starting-branch exit
-> Finite State Machine navigation
-> Unified route-event detection
-> Event cooldown and rearming
-> Ultrasonic obstacle detection
-> C-shaped obstacle avoidance
-> Automatic line reacquisition 
-> False route-event suppression after obstacle avoidance
-> Automatic destination entry
-> End-of-line detection and stopping

# Hardware
1. Parallax Boe-Bot
2. Arduino-compatible controller
3. 2x Continuous Rotation Servos
4. Cytron Maker Line Sensor
5. HC-SR04 Ultrasonic Sensor
6. Battery supply

# Pin Configuration
## Line Sensors
The layout of the line sensors are as followed:

LEFT          RIGHT  
S1  S2  S3  S4  S5

S1  : PIN 6
S2  : PIN 7
S3  : PIN 8
S4  : PIN 9
S5  : PIN 10

## Servo Motors
Left Servo  : PIN 13
Right Servo : PIN 12

## Ultrasonic Sensor
TRIG  : PIN 5
ECHO  : PIN 4

# Destination Selection 
Before the robot begins navigating, the initial line-sensor pattern determines the target destination.

S1  S2  S3  S4  S5
1   0   0   0   0    -> Destination 1
1   1   0   0   0    -> Destination 2
1   1   1   0   0    -> Destination 3

The selected destination is stored as:
targetJunction 
This allows one program to support multiple destinations without uploading separate route programs.

# Software Architecture
The program uses a Finite State Machine (FSM) to separate different navigation behaviors.

<img width="708" height="806" alt="image" src="https://github.com/user-attachments/assets/4ee2059f-0a5a-40a0-983c-555a556b2c1b" />

This structure keeps the Arduino loop() relatively simple while individual behaviors are handled by dedicated functions.

# Line-Following Strategy
The robot continuously reads the line sensors and applies different levels of steering correction.

The line-following controller supports:

-> normal forward movement;
-> soft left correction;
-> soft right correction;
-> hard left correction;
-> hard right correction; and
-> line-loss recovery.

The robot also stores its previous line direction to determine which direction to search if the line is temporarily lost.

# Starting Branch
The robot begins inside a short starting branch.
During this stage, the robot moves toward the junction connecting the starting branch to the main navigation map.
Once the starting junction is detected, the robot performs a compulsory left turn before entering normal navigation.
Keeping this behavior in a dedicated state prevents the starting junction from interfering with normal route-event counting.

# Route-Event Detection 
A unified event counter is used to represent important navigation landmarks:
routeEventCount
Conceptually:

Event 1 -> First compulsory map corner
Event 2 -> Second compulsory map corner
Event 3 -> Destination Junction 1
Event 4 -> Destination Junction 2
Event 5 -> Destination Junction 3

Therefore, the destination event can be determined using:
targetJunction + 2

For instance:
Target Destination = 2
Required Route Event = 2 + 2 = Event 4

# Route-Event Debouncing
Rounded corners presented an important challenge.
Because the robot remains over the wide line while travelling around a rounded corner, the sensors may repeatedly detect the same event.
Without protection:

1 1 1 1 1 -> Event +1
1 1 1 1 1 -> Event +1
1 1 1 1 1 -> Event +1

A single physical corner could therefore be counted multiple times.
The final navigation logic uses:
a. event arming;
b. event cooldown;
c. event rearming; and
d. a stable non-event period.

The detector is only rearmed after the robot has successfully left the previous route event.

# Obstacle Detection 
The HC-SR04 ultrasonic sensor continuously measures the distance in front of the robot.
Distance is calculated using:
Distance = (Echo Duration × Speed of Sound) / 2
The division by two accounts for the ultrasonic pulse travelling to the obstacle and returning to the sensor.
When an obstacle is detected within the configured threshold, normal line following is interrupted and the robot enters the obstacle-avoidance state.

# Obstacle Avoidance
The robot uses a C-shaped bypass maneuver to navigate around an obstacle.
A typical straight-path avoidance sequence is:

<img width="221" height="961" alt="image" src="https://github.com/user-attachments/assets/1a241b4f-8492-4e4d-a84c-3a1ed2433e05" />

This allows obstacles avoidance to temporarily interrupt navigation without resetting the robot's existing route progress.

# Line Reacquisition 
After completing the bypass manoeuvre, the robot approaches the original track approximately perpendicular to the line.
A wide sensor pattern is used to confirm that the track has been found.
The robot then performs a final turn to realign itself with the original direction of travel.

# Preventing False Route Events after Avoidance
One of the major problems discovered during testing was that the line-reacquisition pattern could resemble a genuine route event.
For example, the sensors might detect:
1 1 1 1 1 
while reacquiring the original track.
However, the same wide pattern can also represent a corner or destination junction.
If normal event detection resumed immediately, the robot could incorrectly increment:
routeEventCount

The final solution temporarily disarms route-event detection during obstacle recovery while preserving the existing route count.
Normal event detection is re-enabled only after the robot has successfully returned to normal navigation.

# Engineering Challenges & Solutions

| Challenge | Solution | 
| -------- | -------- | 
| Rounded corners repeatedly triggered route events | Implemented event arming, cooldown, and rearming| 
| Sensor vibration caused inconsistent readings | Redesigned the physical line-sensor mounting | 
| Reacquired line produced false route events | Temporarily disarmed event detection after obstacle recovery |
| Multiple destinations originally required different navigation logic | Implemented sensor-based dynamic destination selection |
| Obstacle avoidance could interrupt route progress | Preserved navigation state and route-event count |
| Line could temporarily be lost during navigation | Implemented direction-aware line searching |
| Hardware behavior differed from expected software behavior | Used isolated hardware tests and Serial Monitor debugging |

# A Key Engineering Lesson
One particularly useful debugging experience occurred when the route-event counter appeared to behave inconsistently.
Serial debugging showed that the software logic was actually operating correctly.
The real cause was mechanical: vibration during extended operation loosened the line-sensor mounting, causing the sensor board to move closer to the track and produce inconsistent readings.
The mounting system was redesigned to maintain a consistent sensor height.
This demonstrated an important principle in mechatronics:
"A problem that appears to originate in software may actually be caused by mechanical or electrical behavior elsewhere in the system."

# Development Process

The robot was developed incrementally:

1. Servo calibration
2. Basic movement testing
3. Line-sensor testing
4. Basic line following
5. Line-loss recovery
6. Starting-branch navigation
7. Route-event detection
8. Event cooldown and rearming
9. Dynamic destination selection
10. Destination entry
11. Ultrasonic sensor testing
12. Straight-path obstacle avoidance
13. Line reacquisition
14. False-event suppression after obstacle recovery
15. Integrated navigation testing
16. Final showcase validation

This incremental approach made it easier to isolate individual hardware and software problems before integrating the complete system.

# Final Results
The completed robot successfully performed autonomous navigation during the final project showcase.
The integrated system demonstrated:

- autonomous line following;
- route recognition;
- dynamic destination selection;
- obstacle detection;
- obstacle avoidance;
- line reacquisition;
- destination identification; and
- autonomous stopping.

# Future Improvements
Several improvements could further develop the system:

1. Replace timing-based movement with wheel encoders
2. Implement closed-loop wheel-speed control
3. Replace blocking delay() operations with non-blocking timing
4. Introduce filtered ultrasonic measurements
5. Implement proportional or PD line-following control
6. Add additional range sensors for more flexible obstacle avoidance
7. Introduce more advanced localization instead of event counting
8. Log navigation telemetry for quantitative performance evaluation

# Technologies and Skills
## Technologies 
Arduino, C++, HC-SR04, Cytron Maker Line Sensor, Parallax Boe-Bot
## Skills
Embedded systems, Autonomous navigation, Finite State Machine, Sensor integration, Obstacle avoidance, Hardware debugging, Robot contorl

# Author
Developed as part of a university Mechatronics Engineering robotics project.
The project focused on the integration of embedded programming, sensor feedback, autonomous navigation, mechanical systems, and systematic engineering debugging.
