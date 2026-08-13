# include <Servo.h>

// Servo Objects
Servo servoLeft;
Servo servoRight;

// Assign pin

// Sensor arrangement
// Left 	Right
// S1 S2 S3 S4 S5

const int S1 = 6;
const int S2 = 7;
const int S3 = 8;
const int S4 = 9;
const int S5 = 10;

const int ECHO_PIN = 4;
const int TRIG_PIN = 5;

// Servo pin
const int Servo_L = 13;
const int Servo_R = 12;

// Navigation configuation & Rouote

// corner & junction couonters
int routeEventCount = 0;
unsigned long lastRouteEventTime = 0;
unsigned long routeEventRearmStartTime = 0;
const unsigned long ROUTE_EVENT_CD_MS = 800;
const unsigned long ROUTE_EVENT_REARM_MS = 1000;
bool routeEventArmed = false;


// Delay moving into junction center before turning
const unsigned long JUNCTION_ENTRY_DELAY_MS = 250;


// Duration of all white readings before cnfirming the end of line
const unsigned long END_LINE_CONFIRMATION_MS = 250;


// Servo calibration

// Stopping
const int STOP_L = 1500;
const int STOP_R = 1500;

// Full speed
const int FORWARD_L = 1700;
const int REVERSE_L = 1300;

const int FORWARD_R = 1300;
const int REVERSE_R = 1700;

// Normal below top speed
const int FORWARD_NORMAL_L = 1650;
const int FORWARD_NORMAL_R = 1350;

// Slow speed for obstacles detection
const int OBSTACLE_SLOW_L = 1580;
const int OBSTACLE_SLOW_R = 1420;

// Soft correction
const int SOFT_L = 1540;
const int SOFT_R = 1460;

// Hard correction
const int HARD_L = 1510;
const int HARD_R = 1490;

// Line-search
const int SEARCH_L_REVERSE = 1470;
const int SEARCH_L_FORWARD = 1600;

const int SEARCH_R_FORWARD = 1400;
const int SEARCH_R_REVERSE = 1530;

// Obstacle detection
const float OBSTACLE_DISTANCE_CM = 12.0;

// Movement calibration
const int LEFT_TURN_L = 1500;
const int LEFT_TURN_R = 1350;

const int RIGHT_TURN_L = 1650;
const int RIGHT_TURN_R = 1500;

const float MS_PER_CM = 63.16;
const float MS_PER_DEG_L = 6.67;
const float MS_PER_DEG_R = 6.67;


// Robot state
enum RobotState{
	SELECT_DESTINATION,
	WAITING_FOR_START_JUNCTION,
	START_LEFT_TURN,
	LINE_FOLLOWING,
	
	OBSTACLES_AVOIDANCE,
	REACQUUIRE_LINE,
	
	DESTINATION_TURN_RIGHT,
	ENTER_DESTINATION,
	ROBOT_STOP
};

RobotState robotState = SELECT_DESTINATION;

// Obstacle avoidance state
enum ObstaclePhase {
	AVOID_START,
	AVOID_FIRST_LEG,
	AVOID_STRAIGHT_SECOND_LEG,
};

ObstaclePhase obstaclePhase = AVOID_START;

// Variables

// -1 = line last detected on Left, 0 = centered, +1 = Right
int last_Direction = 0;


int targetJunction = 0;

bool destination_reached = false;
//bool targetJunctionDetected = false;

unsigned long noLineStartTime = 0;

unsigned long obstacleLegStartTime = 0;

const unsigned long OBSTACLE_FIRST_LEG_MS = (unsigned long)(35 * MS_PER_CM);

bool reacquireNeedsRightTurn = false;


// S1 must stay active this long to confirm a junction
//const unsigned long S5_Confirm_MS = 80;


// Funtion protoype
// State function
void runSelectDestinationState();
void runStartBranchState();
void runStartLeftTurnState();
void runLineFollowingState();
void runObstacleAvidanceState();
void runReacquireLineState();
void runDestinationTurnRightState();
void runEnterDestinationState();
void runRobotStopState();

// Navigation function
void readDestinationSelection();
void checkRouteEvent();
void registerRouteEvent();

// Obstacles
void runObstacleAvidanceState();
void runReacquireLineState();
float getDistanceCM();

// Line followLine
void followLine();
void searchForLine();

// Movement
void moveStraightNormal();
void moveStraightFull();
void moveStraightSlow();
void turnLeftSoft();
void turnRightSOft();
void turnLeftHard();
void turnRightHard();
void searchLeftSoft();
void searchRightSoft();
void pivotLeft();
void pivotRight();
void turnLeft(float angle);
void turnRight(float angle);
void moveForDistance(float distanceCM);
void stopRobot();

// Setup
void setup()
{
	Serial.begin(9600);
	
	servoLeft.attach(Servo_L);
	servoRight.attach(Servo_R);
	
	pinMode(S1, INPUT);
	pinMode(S2, INPUT);
	pinMode(S3, INPUT);
	pinMode(S4, INPUT);
	pinMode(S5, INPUT);
	
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	
	digitalWrite(TRIG_PIN, LOW);
	
	stopRobot();
	delay(5000);
}

void loop()
{
	switch(robotState)
	{
		case SELECT_DESTINATION:
			runSelectDestinationState();
			break;
		
		case WAITING_FOR_START_JUNCTION:
			runStartBranchState();
			break;
		
		case START_LEFT_TURN:
			runStartLeftTurnState();
			break;
		
		case LINE_FOLLOWING:
			runLineFollowingState();
			break;
			
		case OBSTACLES_AVOIDANCE:
			runObstacleAvidanceState();
			break;
		
		case REACQUUIRE_LINE:
			runReacquireLineState();
			break;
		
		case DESTINATION_TURN_RIGHT:
			runDestinationTurnRightState();
			break;
		
		case ENTER_DESTINATION:
			runEnterDestinationState();
			break;
		
		case ROBOT_STOP:
			runRobotStopState();
			break;
	}
}


// Cleaner function call for main loop
void runSelectDestinationState()
{
	stopRobot();
	
	readDestinationSelection();
	
	if (targetJunction > 0)
	{
		Serial.print("Target junction selected: ");
		Serial.println(targetJunction);
		
		delay(3000);
		
		resetNavigation();
		
		robotState = WAITING_FOR_START_JUNCTION;
	}
}

void runStartBranchState()
{
	int s1 = digitalRead(S1);
	int s2 = digitalRead(S2);
	int s3 = digitalRead(S3);
	int s4 = digitalRead(S4);
	int s5 = digitalRead(S5);
	
	bool atStartJunction = s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1;
	
	moveStraightNormal();
	
	if (atStartJunction)
	{
		delay(450);
		stopRobot();
		robotState = START_LEFT_TURN;
		return;
	}
}

void runStartLeftTurnState()
{	
	// Left turn into the map
	turnLeft(90);
	
	// Corner detection is now allowed
	routeEventArmed =  true;
	
	// Prevent starting junction become event 1
	lastRouteEventTime = millis();
	routeEventRearmStartTime = 0;
	
	robotState = LINE_FOLLOWING;
}

void runLineFollowingState()
{
	checkRouteEvent();
	
	if (routeEventCount == targetJunction + 2)
	{
		Serial.println();
		Serial.println("===== TARGET FOUND =====");
		Serial.print("Target Junction = ");
		Serial.println(targetJunction);

		Serial.print("Route Event Count = ");
		Serial.println(routeEventCount);

		Serial.println("========================");
		robotState = DESTINATION_TURN_RIGHT;
		return;
	}
	
	float distance = getDistanceCM();
	
	if (distance > 0 && distance <= OBSTACLE_DISTANCE_CM)
	{
		stopRobot();
		
		Serial.println("Ostacle detected");
		
		robotState = OBSTACLES_AVOIDANCE;
		return;
	}
	
	followLine();
}

void runDestinationTurnRightState()
{
	Serial.println("ENTERED DESTINATION_TURN_RIGHT STATE");
	moveStraightNormal();
	delay(JUNCTION_ENTRY_DELAY_MS);
	
	turnRight(90);
	delay(300);
	
	
	
	last_Direction = 0;
	
	noLineStartTime = 0;
	
	robotState = ENTER_DESTINATION;
}

void runEnterDestinationState()
{
	int s1 = digitalRead(S1);
	int s2 = digitalRead(S2);
	int s3 = digitalRead(S3);
	int s4 = digitalRead(S4);
	int s5 = digitalRead(S5);
	
	bool noLine = s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0;
	
	if (noLine)
	{
		if (noLineStartTime == 0)
		{
			noLineStartTime = millis();
		}
		if (millis() - noLineStartTime >= END_LINE_CONFIRMATION_MS)
		{
			destination_reached = true;
			moveStraightNormal();
			delay(700);
			robotState = ROBOT_STOP;
			return;
		}
	}
	else
	{
		noLineStartTime = 0;
	}
	
	followLine();
}

void runRobotStopState()
{
	stopRobot();
}

// Determine destination 
void readDestinationSelection()
{
	int s1 = digitalRead(S1);
	int s2 = digitalRead(S2);
	int s3 = digitalRead(S3);
	int s4 = digitalRead(S4);
	int s5 = digitalRead(S5);
	
	// Destination 1
	if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)
	{
		targetJunction = 1;
	}
	// Destination 2
	else if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0)
	{
		targetJunction = 2;
	}
	// Destination 3
	else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0)
	{
		targetJunction = 3;
	}
}

// Check for turns and juncitons
void checkRouteEvent()
{
	int s1 = digitalRead(S1);
	int s2 = digitalRead(S2);
	int s3 = digitalRead(S3);
	int s4 = digitalRead(S4);
	int s5 = digitalRead(S5);
	
	unsigned long now = millis();
	
	bool fullEvent = 
	(
		s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1
		||  s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1
	);
	
	if (routeEventArmed && fullEvent && (now - lastRouteEventTime >= ROUTE_EVENT_CD_MS))
	{
		registerRouteEvent();

		Serial.print("Time = ");
		Serial.println(now);

		Serial.print("Sensors = ");
		Serial.print(s1);
		Serial.print(s2);
		Serial.print(s3);
		Serial.print(s4);
		Serial.println(s5);

		return;
	}
	
	if (!routeEventArmed)
	{
		if (!fullEvent)
		{
			if (routeEventRearmStartTime == 0)
			{
				routeEventRearmStartTime = now;
			}
			
			if (now - routeEventRearmStartTime >= ROUTE_EVENT_REARM_MS)
			{
				routeEventArmed = true;
				routeEventRearmStartTime = 0;
			}
		}
		else
		{
			routeEventRearmStartTime = 0;
		}
	}
}

	
// Reset navigationArmed
void resetNavigation()
{
	// Line following
	last_Direction = 0;
	
	// Corner detection
	routeEventCount = 0;
	routeEventArmed = false;
	
	lastRouteEventTime = 0;
	routeEventRearmStartTime = 0;
	
	// Destination state
	//targetJunctionDetected = false;
	destination_reached = false;
	noLineStartTime = 0;
}


// Line-following
void followLine()
{
	int s1 = digitalRead(S1);
	int s2 = digitalRead(S2);
	int s3 = digitalRead(S3);
	
	int activeSensors = s1 + s2 + s3;
	//int error = 0;
	
	if (activeSensors == 0)
	{
		searchForLine();
		return;
	}
	
	// Line checking using left sensors
	if (s1 == 1 && s2 == 1 && s3 == 1)
	{
		moveStraightNormal();
		//last_Direction = 0;
		return;
	}
	
	if (s2 == 1)
	{
		if (s1 == 1 && s3 == 0) 
		{
			turnLeftSoft();
			last_Direction = -1;
		}
		else if(s1 == 0 && s3 == 1)
		{
			turnRightSOft();
			last_Direction = 1;
		}
		else
		{
			moveStraightNormal();
		}
		return;
	}
	
	// Right medium correction
	if (s1 == 1)
	{
		turnLeftHard();
		last_Direction = -1;
		return;
	}
	
	if (s3 == 1)
	{
		turnRightHard();
		last_Direction = 1;
		return;
	}
	
	moveStraightNormal();
};


// Line search
void searchForLine()
{
	if (last_Direction < 0) 
	{
		searchLeftSoft();
	}
	else if (last_Direction > 0)
	{
		searchRightSoft();
	}
	else 
	{
		moveStraightNormal();
	}
}

float getDistanceCM()
{
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	
	digitalWrite(TRIG_PIN, LOW);
	
	unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
	
	if (duration == 0)
	{
		return -1;
	}
	
	return duration * 0.0343 / 2.0;
}


// Obstacle avoidance 
void runObstacleAvidanceState()
{
    int s1 = digitalRead(S1);
	int s2 = digitalRead(S2);
	int s3 = digitalRead(S3);
	int s4 = digitalRead(S4);
	int s5 = digitalRead(S5);
	
	unsigned long now  = millis();
	
	bool wideLine =
	(
		(s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1)
		|| (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1)
		|| (s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1)
	);
	
	switch (obstaclePhase)
	{
		case AVOID_START:
		{
			Serial.println("Avoid: START");
			
			stopRobot();
			delay(500);
			
			turnRight(90);
			moveForDistance(15);
			
			turnLeft(90);
			
			obstacleLegStartTime = millis();
			
			obstaclePhase = AVOID_FIRST_LEG;
			break;
		}
		
		case AVOID_FIRST_LEG:
		{
			moveStraightNormal();
			
			if (wideLine)
			{
				stopRobot();
				Serial.println("Corner Line Detected during Avoidance");
				
				registerRouteEvent();
				
				turnRight(90);
				
				last_Direction = -1;
				
				routeEventArmed = false;
				routeEventRearmStartTime = 0;
				lastRouteEventTime = millis();
				
				obstaclePhase = AVOID_START;
				
				robotState = LINE_FOLLOWING;
				
				return;
			}
			
			if (now - obstacleLegStartTime >= OBSTACLE_FIRST_LEG_MS)
			{
				stopRobot();
				
				Serial.println("No corner line -> Straight obstacles");
				
				turnLeft(90);
				
				reacquireNeedsRightTurn = true;
				
				obstaclePhase = AVOID_START;
				
				robotState = REACQUUIRE_LINE;
				
				return;
			}
			
			break;
		}
	}
}

void runReacquireLineState()
{
	int s1 = digitalRead(S1);
	int s2 = digitalRead(S2);
	int s3 = digitalRead(S3);
	int s4 = digitalRead(S4);
	int s5 = digitalRead(S5);
	
	bool lineDetected =
	(
		s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1
		|| s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1
		|| s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1
	);
	
	if (lineDetected) 
	{
		stopRobot();
		Serial.print("LINE REACQUIRED: ");
		Serial.print(s1);
		Serial.print(s2);
		Serial.print(s3);
		Serial.print(s4);
		Serial.print(s5);
		
		if (reacquireNeedsRightTurn)
		{
			turnRight(90);
			reacquireNeedsRightTurn = false;
		}
		
		last_Direction = -1;
		
		routeEventArmed = false;
		routeEventRearmStartTime = 0;
		lastRouteEventTime = millis();
		
		robotState = LINE_FOLLOWING;
		return;
	}
	
	moveStraightSlow();
}

void registerRouteEvent()
{
	routeEventCount++;
	
	lastRouteEventTime = millis();
	
	routeEventArmed = false;
	routeEventRearmStartTime = 0;
	
	Serial.println("======================");
    Serial.print("ROUTE EVENT = ");
    Serial.println(routeEventCount);
}

// Robot movement
void moveStraightNormal()
{
	servoLeft.writeMicroseconds(FORWARD_NORMAL_L);
	servoRight.writeMicroseconds(FORWARD_NORMAL_R);
}
	
void moveStraightFull()
{
	servoLeft.writeMicroseconds(FORWARD_L);
	servoRight.writeMicroseconds(FORWARD_R);
}

void moveStraightSlow()
{
	servoLeft.writeMicroseconds(OBSTACLE_SLOW_L);
	servoRight.writeMicroseconds(OBSTACLE_SLOW_R);
}

void turnLeftSoft() 
{
	servoLeft.writeMicroseconds(SOFT_L);
	servoRight.writeMicroseconds(FORWARD_NORMAL_R);
}
	
void turnRightSOft()
{
	servoLeft.writeMicroseconds(FORWARD_NORMAL_L);
	servoRight.writeMicroseconds(SOFT_R);
}
	
void turnLeftHard()
{
	servoLeft.writeMicroseconds(HARD_L);
	servoRight.writeMicroseconds(FORWARD_R);
}
	
void turnRightHard()
{
	servoLeft.writeMicroseconds(FORWARD_L);
	servoRight.writeMicroseconds(HARD_R);
}
	
void searchLeftSoft()
{
	servoLeft.writeMicroseconds(SEARCH_L_REVERSE);
	servoRight.writeMicroseconds(SEARCH_R_FORWARD);
}
	
void searchRightSoft()
{
	servoLeft.writeMicroseconds(SEARCH_L_FORWARD);
	servoRight.writeMicroseconds(SEARCH_R_REVERSE);
}
	
void pivotLeft()
{
	servoLeft.writeMicroseconds(REVERSE_L);
	servoRight.writeMicroseconds(FORWARD_R);
}
	
void pivotRight()
{
	servoLeft.writeMicroseconds(FORWARD_L);
	servoRight.writeMicroseconds(REVERSE_R);
}

void turnLeft(float angle)
{
	if (angle < 0)
	{
		angle = -angle;
	}
	
	unsigned long turnTime = (unsigned long)(angle * MS_PER_DEG_L);
	pivotLeft();
	delay(turnTime);
	stopRobot();
}

void turnRight(float angle)
{
	if (angle < 0)
	{
		angle = -angle;
	}
	
	unsigned long turnTime = (unsigned long)(angle * MS_PER_DEG_R);
	pivotRight();
	delay(turnTime);
	stopRobot();
}

void moveForDistance(float distanceCM)
{
	unsigned long moveTime = (unsigned long)(distanceCM * MS_PER_CM);
	moveStraightFull();
	delay(moveTime);
	stopRobot();
}
	
void stopRobot()
{
	servoLeft.writeMicroseconds(STOP_L);
	servoRight.writeMicroseconds(STOP_R);
}
