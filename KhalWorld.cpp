#include "WPILib.h"

using std::string;

class Khaleesi: public IterativeRobot
{
	private:

	//Defense Option Select
	int defenseSelected;

	//Position Option Select
	int positionSelected;

	//auton fire option select
	int autonFireSelected;

	//auton wait option select
	double autonWaitSelected;

	//Shooter PID
	double flyWheelSpeed;
	double flyWheelTarget = 3600;

	bool isAtSpeed = false;

	double intakeArmSpeed = 0;
	double armKP, armKI;

	//Drive Motors
	CANTalon *left, *left2, *right, *right2;

	//Shooter Motors
	CANTalon *intakeInner, *intakeOuter, *flyWheel, *flyWheel2, *arm, *hood;

	//Controllers
	Joystick *driver, *oper;

	//Solenoids
	DoubleSolenoid *rightPTO, *leftPTO;
	Solenoid *climberAngle;
	Solenoid *climbLock;

	//Relay
	Relay *ledSpike;

	//LED bool
	bool ledBool = false;

	//Gyros
	AnalogGyro *horzGyro;
	ADXRS450_Gyro *vertGyro;
	double xAngle = 0, yAngle = 0;
	double target = 0;
	double gyroKP = 0.05;

	bool climbIsReleased = false;//climber is locked
	bool ptoEngaged = true;bool angleSet = false;bool pidArm = false;bool angleReleased = true;bool rampartsFlag = false;

	//Toggle variables for climber
	bool stateAngle = true, releasedAngle = true;bool stateLock = true, releasedLock = true;bool statePTO = false, releasedPTO = true;

	//Toggle variables for anti-tip
	bool stateGyro = true, releasedGyro = true, gyroDisable = false;

	//Banner Sensor for ball detection and Hood Limit Switch
	DigitalInput *bannerInner, *zeroHood;bool ballHit = false;
	DigitalInput *climberSwitch;

	//NetworkTables and Vision variables
	std::shared_ptr<NetworkTable> table;
	std::shared_ptr<NetworkTable> preferences;
	double cogx = 0;
	double cogy = 0;
	double cumcogPIDError = 0;
	double cogPIDError = 0;

	//x PID
	double cogxP = 0.005;
	double cogxI = 0.0009;//.05
	double cogxTar = 190;//TODO: get updated val

	//normPID/Y
	double PIDError = 0;
	double cumPIDError = 0;

	//shooting state machine
	enum shootState
	{
		intake,// Move around and accept a ball
		// Starts 2.5 sec after ball is fired. Ends if ball hits banner sensor
		reving,// Spin firing talon until its fast enough to throw a ball into a goal
		// Starts after ball hits banner, ends if aborted or at 24000 speed
		firing// Ball is sent to the firing talon to fly through the air
	// Starts when reving is at full speed and ends after 2.5 seconds
	};
	shootState currShootState;

	//Timers
	Timer *approachTimer;//auton
	Timer *rampartsTimer;
	Timer *resetFiringState;//teleop shooterPID
	Timer *posTwoTimer;//ummmmm uhhhhh position 2 timer

	//Auton Enums
	enum moatEnum
	{
		moatIntake, moatFirstBump, moatBack, moatSecondBump, moatDrive, moatDrive2, moatRotate, moatShoot
	} moatState;

	enum rockWallEnum
	{
		rockWallIntake, rockWallApproach, rockWallCross, rockWallDrive, rockWallDrive2, rockWallRotate, rockWallShoot
	} rockWallState;

	enum roughTerrainEnum
	{
		roughTerrainIntake, roughTerrainDrive, roughTerrainRotate, roughTerrainShoot
	} roughTerrainState;

	enum rampartsEnum
	{
		rampartsIntake, rampartsRush, rampartsApproach, rampartsCross, rampartsDrive, rampartsRotate, rampartsShoot
	} rampartsState;
	enum spyEnum
	{
		spyIntake, spyRotate, spyShoot
	} spyState;

	enum autonShootState
	{
		autonAligning,//Robot turns left or right in order to see goal
		autonAligning2,//Robot aligns according to X and Y vision
		autonReving,//Shooter begins reving up to fire
		autonFiring//Shooter fires
	} autonShootState;

	enum posTwo
	{
		turn1, forward, turn2
	} posTwoState;
	//Auton SmartDash Variables
	int defense, position;bool autonTimerFlag = true;
	double gyroTar = 0;
	double antiTipAngle = -28;bool antiTipEngaged = false;
	double angleTar = 0;
	//Auton Variables
	double autonTime = 0;//chosen via pos
	bool turnRight = false;//rotate until vision targets
	//Live Stream-Intermediate
	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;

	int myclimb = 0;

	void RobotInit()
	{

		//Gyros
		horzGyro = new AnalogGyro(0);//need to init still
		vertGyro = new ADXRS450_Gyro();

		horzGyro->InitGyro();

		//Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);

		//Drive motors
		left = new CANTalon(14);
		left2 = new CANTalon(15);
		left2->SetControlMode(CANSpeedController::kFollower);
		left2->Set(14);

		right = new CANTalon(12);
		right2 = new CANTalon(13);
		right2->SetControlMode(CANSpeedController::kFollower);
		right2->Set(12);

		//Intake, shooter motors
		intakeInner = new CANTalon(6);
		intakeOuter = new CANTalon(3);

		flyWheel = new CANTalon(0);
		flyWheel->SetFeedbackDevice(CANTalon::QuadEncoder);
		flyWheel->ConfigEncoderCodesPerRev(1000);
		flyWheel->SetControlMode(CANSpeedController::kSpeed);
		flyWheel->SelectProfileSlot(1);

		flyWheel2 = new CANTalon(1);
		flyWheel2->SetControlMode(CANSpeedController::kFollower);
		flyWheel2->Set(0);

		arm = new CANTalon(2);
		hood = new CANTalon(7);

		//Relay
		ledSpike = new Relay(0, Relay::kForwardOnly);

		//Solenoids
		rightPTO = new DoubleSolenoid(0, 3);
		leftPTO = new DoubleSolenoid(1, 2);
		climberAngle = new Solenoid(5);
		climbLock = new Solenoid(4);

		//NetworkTables
		table = NetworkTable::GetTable("SmartDashboard");
		preferences = NetworkTable::GetTable("Preferences");

		//Ball Recog Sensors
		bannerInner = new DigitalInput(0);// 0 means go; 1 means stop

		//Hood Limit Switch
		zeroHood = new DigitalInput(1);// 1 means not pressed; 0 means pressed

		//Climber drive stop
		climberSwitch = new DigitalInput(9);

		//Timer
		approachTimer = new Timer();
		resetFiringState = new Timer();
		rampartsTimer = new Timer();
		posTwoTimer = new Timer();

		//Default auton shoot state
		autonShootState = autonAligning;

		//create an image-Live Stream
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web dashboard
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
		imaqError = IMAQdxConfigureGrab(session);

		//AutonSelect();
		SmartDash();
	}

	void AutonomousInit()
	{
		ledBool = false;
		left->Set(0);
		right->Set(0);

		//Ramparts gyro target
		gyroTar = 0;

		//auton selection
		AutonSelect();

		SmartDash();

		//Auton Timer Reset
		approachTimer->Reset();

		//Select Defense and Position
		leftPTO->Set(DoubleSolenoid::kForward);
		rightPTO->Set(DoubleSolenoid::kReverse);

		defenseChoosing();
		positionChoosing();

		autonShootState = autonAligning;
		posTwoState = turn1;

		//Reset Gyros for Auton Use
		vertGyro->Reset();
		horzGyro->Reset();
		xAngle = 0;
		yAngle = 0;

		//Reset autonTime,rampartFlag, and gyroTar
		autonTime = 0;
		rampartsFlag = false;
		gyroTar = 0;

		//Wait a specified amount of time before auton
		Wait(autonWaitSelected);
	}

	void AutonomousPeriodic()
	{
		SmartDash();
		xAngle = horzGyro->GetAngle();

		switch (defense)
		{
			case 1:
				Moat(position);
				break;
			case 2:
				break;
			case 3:
				RockWall(position);
				break;
			case 4:
				RoughTerrain(position);
				break;
			case 5:
				Ramparts(position);
				break;
			case 6:
				Spy();
		}
	}

	void Moat(int pos)
	{
		switch (pos)
		{
			case 1:
				autonTime = 1.7;//straight pos 2
				break;
			case 2:
				autonTime = 1.7;//curve pos 2
				break;
			case 3:
				autonTime = 1.15;
				angleTar = 22;
				break;
			case 4:
				autonTime = 1.0;
				angleTar = -3;
				break;
			case 5:
				autonTime = 1.8;
				angleTar = -25;
				break;
		}
		switch (moatState)
		{
			case moatIntake:
				ballHit = bannerInner->Get();
				left->Set(0);
				right->Set(0);
				setIntake(1);
				armPID(0);
				if (ballHit == 1)
				{
					setIntake(0);
					moatState = moatFirstBump;
					vertGyro->Reset();
				}
				break;
			case moatFirstBump:
				armPID(0);
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(0.5 + gyroPID(0));
					right->Set(-0.5 + gyroPID(0));
					antiTipEngaged = false;
				}
				if (vertGyro->GetRate() >= 50 && (antiTipEngaged == false))
				{
					left->Set(0);
					right->Set(0);
					Wait(.5);
					approachTimer->Reset();
					approachTimer->Start();
					moatState = moatBack;
				}
				break;
			case moatBack:
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(-.15 + gyroPID(0));
					right->Set(.15 + gyroPID(0));
					antiTipEngaged = false;
				}
				if (approachTimer->Get() >= .75 && (antiTipEngaged == false))
				{
					left->Set(0);
					right->Set(0);
					moatState = moatSecondBump;
					approachTimer->Reset();
					Wait(.5);
				}
				break;
			case moatSecondBump:
				armPID(0);
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(1);
					right->Set(-1);
					antiTipEngaged = false;
				}
				if (vertGyro->GetRate() < -95 && (antiTipEngaged == false))
				{
					left->Set(0);
					right->Set(0);
					if (position == 2)
					{
						Wait(.5);
						posTwoTimer->Start();
						moatState = moatDrive2;
					}
					else
						moatState = moatDrive;
					approachTimer->Reset();
				}
				break;
			case moatDrive:
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(.6 + gyroPID(0));
					right->Set(-.6 + gyroPID(0));
					antiTipEngaged = false;
				}
				if (approachTimer->Get() >= autonTime && (antiTipEngaged == false))
				{
					left->Set(0);
					right->Set(0);
					moatState = moatRotate;
				}
				break;
			case moatDrive2:
				if (secondPos())
					moatState = moatRotate;
				break;

			case moatRotate:
				if (angleTar < 0)
				{
					left->Set(-.5);
					right->Set(-.5);
				}
				else
				{
					left->Set(.5);
					right->Set(.5);
				}
				if (absVal(horzGyro->GetAngle() - angleTar) < 5)
				{
					left->Set(0);
					right->Set(0);
					moatState = moatShoot;
				}
				break;
			case moatShoot:
				autonShoot();
				break;
		}

	}

	void RockWall(int pos)
	{
		switch (pos)
		{
			case 1:
				autonTime = 1.7;//straight pos 2
				break;
			case 2:
				autonTime = 1.7;//curvy pos 2
				break;
			case 3:
				autonTime = 1.15;
				angleTar = 22;
				break;
			case 4:
				autonTime = 1.0;
				angleTar = -3;
				break;
			case 5:
				autonTime = 1.8;
				angleTar = -25;
				break;
		}
		SmartDash();
		switch (rockWallState)
		{
			case rockWallIntake:
				left->Set(0);
				right->Set(0);
				setIntake(1);
				ballHit = bannerInner->Get();
				armPID(0);
				if (ballHit == 1)
				{
					setIntake(0);
					rockWallState = rockWallApproach;
				}
				break;
			case rockWallApproach:
				armPID(0);
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(.65 + gyroPID(0));
					right->Set(-.65 + gyroPID(0));
					antiTipEngaged = false;
				}
				if (vertGyro->GetRate() >= 95 && (antiTipEngaged == false))
				{
					Wait(.3);
					vertGyro->GetRate();
					rockWallState = rockWallCross;
				}
				break;
			case rockWallCross:
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(.8 + gyroPID(0));
					right->Set(-.8 + gyroPID(0));
					antiTipEngaged = false;
				}
				armPID(0);
				if (vertGyro->GetRate() <= -130 && (antiTipEngaged == false))
				{
					left->Set(0);
					right->Set(0);
					Wait(1);
					approachTimer->Reset();
					approachTimer->Start();
					if (position == 2)
					{
						rockWallState = rockWallDrive2;
						posTwoTimer->Start();
					}
					else
						rockWallState = rockWallDrive;
				}
				break;
			case rockWallDrive:
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(.63 + gyroPID(0));
					right->Set(-.63 + gyroPID(0));
					antiTipEngaged = false;
				}
				if (approachTimer->Get() >= autonTime && (antiTipEngaged == false))
				{
					setSpeed(0);
					rockWallState = rockWallRotate;
				}
				break;
			case rockWallDrive2:
				if (secondPos())
					rockWallState = rockWallRotate;
				break;
			case rockWallRotate:
				if (angleTar < 0)
				{
					left->Set(-.5);
					right->Set(-.5);
				}
				else
				{
					left->Set(.5);
					right->Set(.5);
				}
				if (absVal(horzGyro->GetAngle() - angleTar) < 5)
				{
					left->Set(0);
					right->Set(0);

					rockWallState = rockWallShoot;
				}
				break;
			case rockWallShoot:
				autonShoot();
				break;
		}
	}

	void RoughTerrain(int pos)
	{
		switch (pos)
		{
			case 1:
				autonTime = 0;
				break;
			case 2:
				autonTime = 5.8;//4.28=states
				angleTar = 40;
				break;
			case 3:
				autonTime = 4;//4
				angleTar = 22;
				break;
			case 4:
				autonTime = 4.7;//4
				angleTar = -3;
				break;
			case 5:
				autonTime = 5.7;//4.15
				angleTar = -25;
				break;
		}

		switch (roughTerrainState)
		{
			case roughTerrainIntake:
				approachTimer->Reset();
				approachTimer->Start();
				setIntake(1);
				ballHit = bannerInner->Get();
				armPID(0);
				left->Set(0);
				right->Set(0);
				if (ballHit == 1)
				{
					setIntake(0);
					roughTerrainState = roughTerrainDrive;
				}
				break;
			case roughTerrainDrive:
				left->Set(.55 + gyroPID(0));
				right->Set(-.55 + gyroPID(0));
				if (approachTimer->Get() >= autonTime)
				{
					setSpeed(0);
					roughTerrainState = roughTerrainRotate;
				}
				break;
			case roughTerrainRotate:
				if (angleTar < 0)
				{
					left->Set(-.5);
					right->Set(-.5);
				}
				else
				{
					left->Set(.5);
					right->Set(.5);
				}
				if (absVal(horzGyro->GetAngle() - angleTar) < 5)
				{
					left->Set(0);
					right->Set(0);
					roughTerrainState = roughTerrainShoot;
				}
				break;
			case roughTerrainShoot:
				autonShoot();
				break;
		}
	}

	void Ramparts(int pos)
	{
		switch (pos)
		{
			case 1:
				autonTime = 1.6;//straight pos 2
				gyroTar=0;
				break;
			case 2:
				autonTime = 2;//1.6=state; curved pos 2
				gyroTar = 27;
				break;
			case 3:
				autonTime = 1.15;//1.15
				break;
			case 4:
				autonTime = 1.0;//1
				break;
			case 5:
				autonTime = 1.6;
				gyroTar = -65;
				gyroKP = .01;
				angleTar = -10;
				break;
			case 6:
				autonTime = 1.6;
				gyroTar = -20;
				gyroKP = .01;
				angleTar = -10;
				break;
			case 7:
				autonTime = 5.75;
				break;
		}

		switch (rampartsState)
		{
			case rampartsIntake:
				left->Set(0);
				right->Set(0);
				approachTimer->Reset();
				approachTimer->Start();
				setIntake(1);
				ballHit = bannerInner->Get();
				armPID(0);
				if (ballHit == 1 && pos != 7)
				{
					setIntake(0);
					rampartsState = rampartsApproach;
				}
				else if(ballHit == 1)
				{
					setIntake(0);
					rampartsState = rampartsRush;
				}
				break;
			case rampartsRush:
				if (approachTimer->Get() < autonTime)
				{
					left->Set(.7);
					right->Set(-.7);
				}
				else
				{
					left->Set(0);
					right->Set(0);
					rampartsState = rampartsShoot;
				}
				break;
			case rampartsApproach:
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(.7);
					right->Set(-.7);
					antiTipEngaged = false;
				}
				if (vertGyro->GetRate() <= -75 && (antiTipEngaged == false))
				{
					Wait(.3);
					vertGyro->GetRate();
					rampartsState = rampartsCross;
				}
				if (approachTimer->Get() >= 5.5)
				{
					left->Set(0);
					right->Set(0);
					rampartsState = rampartsRotate;
				}
				break;
			case rampartsCross:
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(.55);
					right->Set(-.75);
					antiTipEngaged = false;
				}
				if (vertGyro->GetRate() >= 75 && (antiTipEngaged == false))
				{
					rampartsFlag = true;
					vertGyro->GetRate();
				}
				if (vertGyro->GetRate() <= -75 && rampartsFlag && (antiTipEngaged == false))
				{
					vertGyro->GetRate();
					left->Set(0);
					right->Set(0);
					Wait(.75);
					rampartsState = rampartsDrive;
					approachTimer->Reset();
					approachTimer->Start();
				}
				if (approachTimer->Get() >= 4.25)
				{
					left->Set(0);
					right->Set(0);
					rampartsState = rampartsRotate;
				}
				break;
			case rampartsDrive:
				if (vertGyro->GetAngle() < antiTipAngle)
				{
					left->Set(-.4);
					right->Set(.4);
					antiTipEngaged = true;
				}
				else
				{
					left->Set(.7 + gyroPID(gyroTar));
					right->Set(-.7 + gyroPID(gyroTar));
					antiTipEngaged = false;
				}
				if (approachTimer->Get() > autonTime && (antiTipEngaged == false))
				{
					left->Set(0);
					right->Set(0);
					if (pos == 5 || pos == 6)
					{
						rampartsState = rampartsRotate;
						rampartsTimer->Reset();
						rampartsTimer->Start();
					}
					else
					{
						rampartsState = rampartsShoot;
					}
				}
				break;
			case rampartsRotate:
				if (angleTar < 0 && pos != 5)
				{
					left->Set(-.5);
					right->Set(-.5);
				}
				else
				{
					left->Set(.5);
					right->Set(.5);
				}
				if (absVal(horzGyro->GetAngle() - angleTar) < 5)
				{
					left->Set(0);
					right->Set(0);
					rampartsState = rampartsShoot;
				}
				break;
			case rampartsShoot:
				autonShoot();
				break;
		}
	}

	//linear turn auton on second position defense
	bool secondPos()
	{
		switch (posTwoState)
		{
			case turn1:
				left->Set(.5);
				right->Set(.5);
				if (absVal(horzGyro->GetAngle() - 45) < 5.0)
				{
					posTwoState = forward;
					posTwoTimer->Reset();
				}
				return false;
				break;
			case forward:
				left->Set(.67 + gyroPID(45));
				right->Set(-.67 + gyroPID(45));
				if (posTwoTimer->Get() > autonTime)
					posTwoState = turn2;
				return false;
				break;
			case turn2:
				left->Set(-.33);
				right->Set(-.33);
				if (absVal(horzGyro->GetAngle()) < 5.0)
					return true;
				else
					return false;
				break;
		}
		return false;
	}
	void Spy()
	{
		cogx = table->GetNumber("COG_X", 1000);
		cogy = table->GetNumber("COG_Y", 1000);
		switch (spyState)
		{
			case spyIntake:
				approachTimer->Reset();
				approachTimer->Start();
				setIntake(1);
				ballHit = bannerInner->Get();
				armPID(0);
				if (ballHit == 1)
				{
					setIntake(0);
					spyState = spyRotate;
				}
				break;
			case spyRotate:
				//right->Set(-.3);
				visionXAlign();
				if ((cogx < (cogxTar + 5) && cogx > (cogxTar - 5)) && visionPID(cogxTar, cogx) == 0)///approachTimer->Get() >= 1
				{

					setSpeed(0);
					spyState = spyShoot;
				}
				break;
			case spyShoot:
				autonShootState = autonReving;
				autonShoot();
				break;
		}
	}

	//State machine for aligning and firing in auton
	void autonShoot()
	{
		cogx = table->GetNumber("COG_X", 1000);
		cogy = table->GetNumber("COG_Y", 1000);

		if (autonFireSelected)
		{
			switch (autonShootState)
			{
				case autonAligning:
					switch (position)
					{
						case 1:
						case 2:
						case 3:
							turnRight = true;
							break;
						case 4:
						case 6:
							turnRight = false;
							break;
						case 5:
							if (defense == 5)
								turnRight = true;
							else
								turnRight = false;
							break;
						default:
							break;
					}
					if (turnRight && cogx == 0)
					{
						left->Set(.5);
						right->Set(.5);
					}
					else if (!turnRight && cogx == 0)
					{
						left->Set(-.5);
						right->Set(-.5);
					}
					else
					{
						left->Set(0);
						right->Set(0);
						autonShootState = autonAligning2;
					}
					break;
				case autonAligning2:
					visionXAlign();
					//stops hood from going past point
					if (hood->GetEncPosition() < -1650)
					{
						hood->Set(normPID(-1300, hood->GetEncPosition(), .0048, 0));
					}
					else
						visionYAlign();
					if (cogx < (cogxTar + 5) && cogx > (cogxTar - 5) && hood->GetSpeed() == 0)
					{
						autonShootState = autonReving;
					}
					break;
				case autonReving:
					visionXAlign();
					//stops hood from going past point
					if (hood->GetEncPosition() < -1650)
					{
						hood->Set(normPID(-1300, hood->GetEncPosition(), .0048, 0));
					}
					else
						visionYAlign();

					setFlyWheelRPM(flyWheelTarget);

					if (isAtSpeed)
					{
						// switch states because we are completely reved up
						resetFiringState->Reset();
						resetFiringState->Start();
						autonShootState = autonFiring;

					}
					break;
				case autonFiring:
					if (resetFiringState->Get() < 1)
					{
						setIntake(1);
					}
					else
					{
						setIntake(0);
						stopFlyWheel();
					}
					// If the ball is still at the banner sensor, push it into the shooter

					break;
			}
		}
		else
		{
			left->Set(0);
			right->Set(0);
		}
	}

	void disableGyro()
	{
		//manual disable antitip
		if (driver->GetRawButton(13) && !stateGyro)//after second push
		{
			releasedGyro = false;
			gyroDisable = false;//Enable anti-tip
		}
		else if (driver->GetRawButton(13) && stateGyro)//first push
		{
			releasedGyro = false;
			gyroDisable = true;//disable anti-tip
		}
		else if (!driver->GetRawButton(13) && !releasedGyro)//happens first
		{
			stateGyro = !stateGyro;
			releasedGyro = true;
		}
	}
	void TeleopInit()
	{

		flyWheel->SetEncPosition(0);

		SmartDash();
		approachTimer->Stop();

		//Live Stream
		IMAQdxStartAcquisition(session);

		//ensure solenoids are disengaged
		climberAngle->Set(0);
		climbLock->Set(0);

		leftPTO->Set(DoubleSolenoid::kForward);
		rightPTO->Set(DoubleSolenoid::kReverse);
		ptoEngaged = true;
		statePTO = false;
		gyroDisable = false;

	}

	void TeleopPeriodic()
	{
		disableGyro();
		SmartDash();

		flyWheelSpeed = flyWheel->GetSpeed();

		if (driver->GetRawButton(7))
		{
			visionXAlign();
		}
		else
		{
			if (vertGyro->GetAngle() < antiTipAngle && ptoEngaged == true && !gyroDisable)//pto is not engaged
			{
				left->Set(-.65);
				right->Set(.65);
			}
			else
			{
				Drivebase();
			}
		}
		if (oper->GetRawButton(10))
		{
			flyWheel->SetControlMode(CANSpeedController::kPercentVbus);
			flyWheel->Set(1);
			if (oper->GetRawButton(14))
			{
				setIntake(1);
			}
			else if (!oper->GetRawButton(14))
			{
				setIntake(0);
			}
		}
		else
		{
			Shooter();
		}
		Gyro();
		Arm();
		//stops hood from going past point
		if (hood->GetEncPosition() < -1650)
		{
			hood->Set(normPID(-1300, hood->GetEncPosition(), .0048, 0));
		}
		else
			Hood();
		Climber();
		LiveStream();
	}
	void DisabledInit()
	{
		ledSpike->Set(Relay::kOff);
		ledBool = false;
	}
	void DisabledPeriodic()
	{
		if (climberSwitch->Get() == 0)
		{
			vertGyro->Calibrate();
			horzGyro->Calibrate();
		}
	}

	void TestPeriodic()
	{

	}

//Function to display various data to SmartDashboard
	void SmartDash()//TODO: find what evan wants
	{

		string cogxString = "COG_X " + std::to_string(table->GetNumber("COG_X", 1000));
		SmartDashboard::PutString("DB/String 0", cogxString);

		string cogyString = "COG_Y " + std::to_string(table->GetNumber("COG_Y", 1000));
		SmartDashboard::PutString("DB/String 1", cogyString);

		string pto;

		//Driving mode
		if (ptoEngaged == 1)
		{
			pto = "true";
			SmartDashboard::PutBoolean("DB/LED 1", !ptoEngaged);
		}
		//Climbing mode
		if (ptoEngaged == 0)
		{
			pto = "false";
			SmartDashboard::PutBoolean("DB/LED 1", !ptoEngaged);
		}
//		string PTOString = "PTO: " + pto;
//		SmartDashboard::PutString("DB/String 2", PTOString);

		string bann;

		//Ball held
		if (bannerInner->Get())
		{
			bann = "true";
			SmartDashboard::PutBoolean("DB/LED 0", ballHit);
		}
		//No Ball
		if (!bannerInner->Get())
		{
			SmartDashboard::PutBoolean("DB/LED 0", ballHit);
			bann = "false";
		}
//		string ballHitString = "Ball Held: " + bann;
//		SmartDashboard::PutString("DB/String 3", ballHitString);

		string HoodString = "Hood: " + std::to_string(hood->GetEncPosition());
		SmartDashboard::PutString("DB/String 4", HoodString);

//		SmartDashboard::PutString("DB/String 2", std::to_string(horzGyro->GetAngle()));

		string armString = "Arm: " + std::to_string(arm->GetEncPosition());
		SmartDashboard::PutString("DB/String 3", armString);
//		string shooterString = "flywheel: "
//				+ std::to_string(flyWheel->GetSpeed());
//		SmartDashboard::PutString("DB/String 2", shooterString);

		//string horzString = "horzgyro: " + std::to_string(horzGyro->GetAngle());
		//SmartDashboard::PutString("DB/String 2", horzString);

		string gyroString = "VertGyro: " + std::to_string(vertGyro->GetAngle());
		SmartDashboard::PutString("DB/String 2", gyroString);

//		SmartDashboard::PutString("DB/String 2",
//				(absVal(flyWheelSpeed) > 40
//						&& (oper->GetRawButton(10) || oper->GetRawButton(4))) ?
//						"Shooting" : "Standby");

		//SmartDashboard::PutString("DB/String 2", PTOString);

	}

	void AutonSelect()
	{
		string defString = SmartDashboard::GetString("DB/String 5", "2");
		defenseSelected = std::stoi(defString);//what defense

		string posString = SmartDashboard::GetString("DB/String 6", "2");
		positionSelected = std::stoi(posString);//what position

		string autonFireString = SmartDashboard::GetString("DB/String 7", "1");
		autonFireSelected = std::stoi(autonFireString);

		string autonWaitString = SmartDashboard::GetString("DB/String 8", "0");
		autonWaitSelected = std::stod(autonWaitString);
	}

	//Function to capture images and send to DashBoard
	void LiveStream()
	{
		IMAQdxStartAcquisition(session);
		//Acquire Images
		IMAQdxGrab(session, frame, true, NULL);

		//Send Image to DashBoard
		CameraServer::GetInstance()->SetImage(frame);
	}

//Function for operating drivebase. Allows for tank/Arcade option select
//Includes gyro correction. Must be tested
	void Drivebase()
	{
		double speed = deadband(-driver->GetY());
		double turn = deadband(driver->GetZ());

		if (!driver->GetRawButton(8))
		{
			target = xAngle;
		}

		if (climberSwitch->Get() == 0 && !ptoEngaged)
		{
			left->Set(0);
			right->Set(0);
		}

		else
		{
			left->Set(speed + turn + gyroPID(target));
			right->Set(-speed + turn + gyroPID(target));
		}
	}
//Updates angle variables, resets x gyro when full rotation is completed
	void Gyro()
	{
		xAngle = horzGyro->GetAngle();
		yAngle = vertGyro->GetAngle();
		if (xAngle == 360 || xAngle == -360)
		{
			xAngle = 0;
		}
		if (driver->GetRawButton(6))//TODO: confirm evan
		{
			horzGyro->Reset();
			vertGyro->Reset();
		}
	}

//Function for operating shooter
	void Hood()
	{

		//stops hood from going past point
		if (hood->GetEncPosition() < -1650)
		{
			hood->Set(normPID(-1300, hood->GetEncPosition(), .0048, 0));
		}
		else
		{
			//reset hood when its all the way down
			if (zeroHood->Get() == 0)
			{

				if (oper->GetRawButton(6) == 1)
				{
					hood->Set(0);
				}
				hood->SetEncPosition(0);
				hood->Set(normPID(50, hood->GetEncPosition(), .0048, 0));
			}
			else if (driver->GetRawButton(5))//move hood up
			{
				visionYAlign();
			}
			else
			{
				if (oper->GetRawButton(6))
				{
					hood->Set(.75);
				}
				else if (oper->GetRawButton(8))
				{
					hood->Set(-.75);
				}
				else
					hood->Set(0);
			}

			//Far far shot (position 2 outer works)
			if (oper->GetRawButton(9))
			{
				hood->Set(normPID(640, hood->GetEncPosition(), .0048, 0));//610
				flyWheelTarget = 4050;
			}
			//lob shot, arm out
			if (oper->GetPOV() == 0)
			{
				hood->Set(normPID(815, hood->GetEncPosition(), .0048, 0));
				flyWheelTarget = 3315;
			}
			//far shot
			if (oper->GetPOV() == 90)
			{
				hood->Set(normPID(624, hood->GetEncPosition(), .0048, 0), 0);//679
				flyWheelTarget = 3950;
			}
			//mid shot
			if (oper->GetPOV() == 180)
			{
				hood->Set(normPID(655, hood->GetEncPosition(), .0048, 0), 0);//660
				flyWheelTarget = 3600;
			}
			//close shot, edge of batter
			if (oper->GetPOV() == 270)
			{
				hood->Set(normPID(835, hood->GetEncPosition(), .0048, 0), 0);//855
				flyWheelTarget = 3600;
			}
			//Batter shot, fully up touching tower
			if (oper->GetRawButton(14) && !oper->GetRawButton(10))
			{
				hood->Set(normPID(1125, hood->GetEncPosition(), .0048, .0025));//1320=hard,1340=soft
				flyWheelTarget = 3000;//3600=hard,3000=soft
			}
		}
	}

//state machine for operating shooter
	void Shooter()
	{
		/*
		 * 1. Operator uses intake to claim a ball and can spit out as well.
		 * 	Arm comes down automatically.
		 * 2. Banner sensor is hit to stop ball. Ball can be spit out.
		 * 	Arm comes all the way up.
		 * 3. Operator revs up shooter and holds intake, intake is not activated
		 * 	until shooter is up to speed.
		 */
		ballHit = bannerInner->Get();
		if (ballHit || ledBool)
		{
			ledSpike->Set(Relay::kOn);
		}
		else
		{
			ledSpike->Set(Relay::kOff);
		}
		switch (currShootState)
		{

			case intake:
				stopFlyWheel();
				isAtSpeed = false;

				// In this mode, the robot is waiting to obtain a ball
				if (oper->GetRawButton(5))
				{
					setIntake(-1);

					// If the regurgitation button is pressed, pull a ball in
				}
				else if (oper->GetRawButton(7))
				{
					setIntake(1);
					if (!oper->GetRawButton(3))
					{
						pidArm = true;
						armPID(-1250);//TODO: ask Dylan
					}
					// If the intake button is pressed, spit the ball out
				}
				else
				{
					setIntake(0);
					pidArm = false;
				}

				if (ballHit == 1)
				{
					// switch states because we've fully loaded a ball
					setIntake(0);
					currShootState = reving;
				}

				break;
			case reving:
				// In this mode, the robot is waiting for the shooter to reach 24K speed
				if (oper->GetRawButton(7) && !oper->GetRawButton(3))
				{
					armPID(0);
					pidArm = true;
				}
				else
				{
					pidArm = false;
				}

				if (oper->GetRawButton(5))
				{
					setIntake(-1);
					currShootState = intake;
					// Regurgitate, and go to intake because we're spitting out the ball

				}
				else if (oper->GetRawButton(2))
				{
					intakeOuter->Set(-1);

				}
				else if (!oper->GetRawButton(2))
				{
					intakeOuter->Set(0);
				}
				if (oper->GetRawButton(4))
				{
					setFlyWheelRPM(flyWheelTarget);
					// This runs the rev up process, PID'ing the shooter to 24000.
				}
				else
				{
					stopFlyWheel();
				}

				if (isAtSpeed && oper->GetRawButton(7))//isAtSpeed
				{
					// switch states because we are completely reved up
					currShootState = firing;

					resetFiringState->Reset();
					resetFiringState->Start();
				}
				break;
			case firing:
				if (ballHit == 1)
				{
					// If the ball is still at the banner sensor, push it into the shooter
					intakeInner->Set(-1);
				}
				if (resetFiringState->Get() >= 1.5)
				{
					currShootState = intake;
				}
				break;
		}

	}
	void setFlyWheelRPM(double rpm)
	{
		flyWheel->SetControlMode(CANSpeedController::kSpeed);
		flyWheel->Set(rpm);
		flyWheelSpeed = flyWheel->GetSpeed();
		double flyWheelError = absVal(rpm - flyWheelSpeed) - 360;
		if (flyWheelError <= 500)
		{
			isAtSpeed = true;
		}
		else
		{
			isAtSpeed = false;
		}
	}
	void stopFlyWheel()
	{
		flyWheel->SetControlMode(CANSpeedController::kPercentVbus);
		flyWheel->Set(0);
	}

//function for operating arm
	void Arm()
	{
		/*
		 * Arm comes up full speed, down half speed (0.5)
		 *
		 * Square - arm comes up to 0
		 * X - arm becomes level with floor
		 */

		//left stick
		intakeArmSpeed = deadband(oper->GetY());
		if (arm->GetEncPosition() > -650 && intakeArmSpeed < 0)
		{
			intakeArmSpeed = 0;
		}
		if (oper->GetRawButton(13))
		{
			arm->SetEncPosition(0);

		}

		if ((deadband(oper->GetRawAxis(5)) != 0) && (deadband(oper->GetRawAxis(5)) < 0))//move intake arm up or down
		{
			arm->Set(deadband(oper->GetRawAxis(5)) * .8);
		}
		else if (intakeArmSpeed < 0)
		{
			arm->Set(intakeArmSpeed * .5);
			pidArm = false;
		}
		else if (intakeArmSpeed > 0)
		{
			arm->Set(intakeArmSpeed * 0.3);
			pidArm = false;
		}
		else if (oper->GetRawButton(3))
		{
			armPID(0);
		}
		else if (oper->GetRawButton(1))
		{
			armPID(-1000);
		}
		else if (oper->GetRawButton(2))
		{
			armPID(-1250);
		}
		else if (!pidArm)
		{
			arm->Set(0);
		}
	}

//Sets speeds of both drive motors to for/back
	void Climber()
	{
//		string climbString = "climb: " + std::to_string(myclimb);
//		SmartDashboard::PutString("DB/String 2", climbString);
		/*
		 * Square - Toggle for angle
		 * X - Toggle for release
		 * Circle - Toggle for PTOs
		 */

		//climber angle Toggle
		if (driver->GetRawButton(1) && !stateAngle)//after second push
		{
			releasedAngle = false;
			climberAngle->Set(0);//becomes 0 degrees
			myclimb = 1;

		}
		else if (driver->GetRawButton(1) && stateAngle)//first push
		{
			releasedAngle = false;
			climberAngle->Set(1);//becomes 30 degrees
			myclimb = 2;

		}
		else if (!driver->GetRawButton(1) && !releasedAngle)//happens first
		{
			stateAngle = !stateAngle;
			releasedAngle = true;
			myclimb = 3;
		}

		//climber release Toggle
		if (driver->GetRawButton(2) && !stateLock)//after second push
		{
			releasedLock = false;
			//climbLock->Set(0);//re-engages lock
			myclimb = 4;

		}
		else if (driver->GetRawButton(2) && stateLock)//first push
		{
			releasedLock = false;
			climbLock->Set(1);//releases lock
			ledBool = true;
			myclimb = 5;
		}
		else if (!driver->GetRawButton(2) && !releasedLock)//happens first
		{
			stateLock = !stateLock;
			releasedLock = true;
			myclimb = 6;
		}

		//climberPTO Toggle
		if (driver->GetRawButton(3) && !statePTO)//after second push
		{
			releasedPTO = false;
			ptoEngaged = false;
			leftPTO->Set(DoubleSolenoid::kReverse);
			rightPTO->Set(DoubleSolenoid::kForward);//climb mode
			Wait(.25);
			releasedAngle = false;
			climberAngle->Set(0);//becomes 0 degrees
			myclimb = 7;

		}
		else if (driver->GetRawButton(3) && statePTO)//first push
		{
			releasedPTO = false;
			ptoEngaged = true;
			leftPTO->Set(DoubleSolenoid::kForward);
			rightPTO->Set(DoubleSolenoid::kReverse);//drive mode
			myclimb = 8;

		}
		else if (!driver->GetRawButton(3) && !releasedPTO)//happens first
		{
			statePTO = !statePTO;
			releasedPTO = true;
			myclimb = 9;
		}

	}

//PID for x-axis alignment
	void visionXAlign()
	{
		cogx = table->GetNumber("COG_X", 1000);
		left->Set(-visionPID(cogxTar, cogx));
		right->Set(-visionPID(cogxTar, cogx));
	}

//PID for hood alignment
//TODO: find actual values for this
	void visionYAlign()
	{
		cogy = table->GetNumber("COG_Y", 217);
		if (cogy < 28)//backed up to defenses at courtyard
		{
			hood->Set(normPID(640, hood->GetEncPosition(), .0048, 0));
		}
		else if (cogy < 55)
		{
			hood->Set(normPID(640, hood->GetEncPosition(), .0048, 0));
		}
		else if (cogy < 82)
		{
			hood->Set(normPID(640, hood->GetEncPosition(), .0048, 0));
		}
		else if (cogy < 110)
		{
			hood->Set(normPID(650, hood->GetEncPosition(), .0048, 0));
		}
		else if (cogy < 140)
		{
			hood->Set(normPID(750, hood->GetEncPosition(), .0048, 0));
		}
		else if (cogy < 165)
		{
			hood->Set(normPID(830, hood->GetEncPosition(), .0048, 0));
		}
		else
			hood->Set(normPID(880, hood->GetEncPosition(), .0048, 0));

	}

	double gyroPID(double target)
	{
		double gyroError;

		gyroError = target - xAngle;
		return gyroError * gyroKP;

	}

//PID for x-axis vision alignment use
//TODO: edit add like .15 to everything to mimic amber
	double visionPID(double cogTar, double cogPos)
	{
		cogPIDError = cogTar - cogPos;
		double cogPIDSpeed;

		if (cogPIDError > 40)
		{
			cogPIDSpeed = .38;//was .37
		}
		else if (cogPIDError > 5)
		{
			cogPIDSpeed = .34;//was .27
		}
		else if (cogPIDError < -40)
		{
			cogPIDSpeed = -.38;//was -.37
		}
		else if (cogPIDError < -5)
		{
			cogPIDSpeed = -.34;//-.27
		}
		else
		{
			cogPIDSpeed = 0;
		}
		return (cogPIDSpeed);
	}

//PID for arm
	void armPID(double armTar)
	{
		double armCur = arm->GetEncPosition();
		double armSpeed = -normPID(armTar * 4, -armCur, .00015, 0);
		if (armSpeed > .5)
		{
			armSpeed = .5;
		}
		if (armSpeed < -.5)
		{
			armSpeed = -.5;
		}
		arm->Set(armSpeed);

	}

//Standard PID function
	double normPID(double myTar, double myPos, double myP, double myI)
	{
		PIDError = myTar + myPos;
		cumPIDError = PIDError;
		double PIDPout = PIDError * myP;
		double PIDIout = PIDError * myI;
		double PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDError) < 5)
		{
			PIDSpeed = 0;
		}
		return (PIDSpeed);
	}

//Sets all wheels to drive at same speed
	void setSpeed(double speed)
	{
		left->Set(speed);
		right->Set(-speed);
	}

//Sets speed of intake motors
	void setIntake(double speed)
	{
		intakeInner->Set(-speed);
		intakeOuter->Set(speed);
	}

//Nulls idle stick input at 0.08
	double deadband(double input)
	{

		if (absVal(input) < .08)
		{
			return 0;
		}
		return input;
	}

//choosing position for auton
	void positionChoosing()
	{
		if (positionSelected == 1)
			position = 1;
		if (positionSelected == 2)
			position = 2;
		if (positionSelected == 3)
			position = 3;
		if (positionSelected == 4)
			position = 4;
		if (positionSelected == 5)
			position = 5;
		if (positionSelected == 6)
			position = 6;
		if(positionSelected == 7)
			position = 7;
	}

//choosing defense to cross in auton
	void defenseChoosing()
	{
		if (defenseSelected == 1)
		{
			defense = 1;
			moatState = moatIntake;
		}
		if (defenseSelected == 2)
		{
			defense = 2;
		}
		if (defenseSelected == 3)
		{
			defense = 3;
			rockWallState = rockWallIntake;
		}
		if (defenseSelected == 4)
		{
			defense = 4;
			roughTerrainState = roughTerrainIntake;
		}
		if (defenseSelected == 5)
		{
			defense = 5;
			rampartsState = rampartsIntake;
		}
		if (defenseSelected == 6)
		{
			defense = 6;
			spyState = spyIntake;
		}
	}

//C++ abs function sucks
	double absVal(double input)
	{
		if (input < 0)
			return -input;
		return input;
	}

};

START_ROBOT_CLASS(Khaleesi)
