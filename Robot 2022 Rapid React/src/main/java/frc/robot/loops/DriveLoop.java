package frc.robot.loops;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Subsystems.Subsystems.Drive;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.lib.sensors.GyroBase;
import frc.robot.lib.sensors.Pigeon;

/*
 * DriveLoop is the interface between Drive.java and the actual hardware.
 * It runs periodically, taking the commands sent by Drive and sending them to the hardware.
 * In this way, Drive.java does not access the hardware directly.  The benefits of this partition are: 
 * 1) Changes to drive hardware only requires changes to DriveLoop, not Drive
 * 2) DriveLoop can be easily replaced for simulation purposes.
 */

public class DriveLoop implements Loop 
{
 	// singleton class
	 private static DriveLoop instance = null;
	 public static DriveLoop getInstance() 
	 { 
		 if (instance == null) {
			 instance = new DriveLoop();
		 }
		 return instance;
	 }
	 
    private static Drive drive;
	private static GyroBase gyro;
    private DriveState driveState;
    
	public final TalonFX lMotorMaster;
	public final TalonFX rMotorMaster;
	public final List<BaseMotorController> lMotorSlaves;
	public final List<BaseMotorController> rMotorSlaves;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;

	// Motor Controller Inversions
	
    public static TalonFXInvertType kLeftMotorInverted = TalonFXInvertType.CounterClockwise;
    public static TalonFXInvertType kRightMotorInverted = TalonFXInvertType.Clockwise;

    public static int kDriveTrainCurrentLimit = 25;

	// Constant import
	public static int kTalonTimeoutMs = 5;
	public static int kTalonPidIdx = 0;

	public enum GyroSelectionEnum { BNO055, NAVX, PIGEON; }
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;

	public static double kDriveWatchdogTimerThreshold = 0.500;

	// Wheels
	public static double kDriveWheelCircumInches    = 6*Math.PI;
	public static double kTrackLengthInches         = 17.500;
	public static double kTrackWidthInches          = 26.000;
	public static double kTrackEffectiveDiameter    = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;;
	public static double kTrackScrubFactor          = 0.5;

	// Wheel Encoder
	public static int    kFalconEncoderUnitsPerRev    = 2048;
	public static double kDriveGearRatio				= 50/14*50/14;
	public static double kFalconEncoderStatusFramePeriod = 0.100;	// 100 ms

	// CONTROL LOOP GAINS   
	public static double kCalEncoderPulsePer100ms = 1400;		// velocity at a nominal throttle (measured using NI web interface)
	public static double kCalPercentOutput 		 = 0.49;	// percent output of motor at above throttle (using NI web interface)
   
   // CONTROL LOOP GAINS
   public static double kFullThrottlePercentOutput = 1.0;	
   public static double kFullThrottleEncoderPulsePer100ms = 2900; 

    // PID gains for drive velocity loop (sent to Talon)
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 0.3;//1.0;
    public static double kDriveVelocityKi = 0.00;
    public static double kDriveVelocityKd = 5.0;//5.0;
    public static double kDriveVelocityKf = kCalPercentOutput * 1023.0 / kCalEncoderPulsePer100ms;
    public static int    kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.375;	// seconds from zero to full speed
    public static int    kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int    kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int    kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 4.0;//4.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 0.0;//50.0;
    
    // Point Turn constants
    public static double kPointTurnKp = 0.05;
    public static double kPointTurnKd = 0.50;
    public static double kPointTurnKi = 0.00;
    public static double kPointTurnKf = 0.00;
    public static double kPointTurnCompletionToleranceDeg = 3.0;
    public static double kPointTurnMaxOutput = 0.7; 
    
    // Path following constants
    public static double kPathFollowingMaxVel    = 72.0; // inches/sec  		
    public static double kPathFollowingAccelTime = 0.5;		// sec to reach max velocity
    public static double kPathFollowingMaxAccel  = kPathFollowingMaxVel / kPathFollowingAccelTime; // inches/sec^2
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingCompletionTolerance = 4.0; 



	private DriveLoop() 
	{
		drive = Drive.getInstance();
		driveState = DriveState.getInstance();
		
		/*****************************************************************
		 * Configure Master Motor Controllers
		 *****************************************************************/
		lMotorMaster = new TalonFX(Constants.kLeftMasterID);
		rMotorMaster = new TalonFX(Constants.kRightMasterID);
        
		lMotorMaster.configFactoryDefault();
		rMotorMaster.configFactoryDefault();

		// Get status at 100Hz (faster than default 50 Hz)
		lMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, kTalonTimeoutMs);
		rMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, kTalonTimeoutMs);

		lMotorMaster.set(ControlMode.PercentOutput, 0.0);
		rMotorMaster.set(ControlMode.PercentOutput, 0.0);
		lMotorMaster.setNeutralMode(NeutralMode.Coast);
		rMotorMaster.setNeutralMode(NeutralMode.Coast);

		// Set up the encoders
		lMotorMaster.setInverted(kLeftMotorInverted);
		rMotorMaster.setInverted(kRightMotorInverted);
		
		// Load velocity control gains
		lMotorMaster.config_kF(kVelocityControlSlot, kDriveVelocityKf, kTalonTimeoutMs);
		lMotorMaster.config_kP(kVelocityControlSlot, kDriveVelocityKp, kTalonTimeoutMs);
		lMotorMaster.config_kI(kVelocityControlSlot, kDriveVelocityKi, kTalonTimeoutMs);
		lMotorMaster.config_kD(kVelocityControlSlot, kDriveVelocityKd, kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kVelocityControlSlot, kDriveVelocityIZone, kTalonTimeoutMs);

		rMotorMaster.config_kF(kVelocityControlSlot, kDriveVelocityKf, kTalonTimeoutMs);
		rMotorMaster.config_kP(kVelocityControlSlot, kDriveVelocityKp, kTalonTimeoutMs);
		rMotorMaster.config_kI(kVelocityControlSlot, kDriveVelocityKi, kTalonTimeoutMs);
		rMotorMaster.config_kD(kVelocityControlSlot, kDriveVelocityKd, kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kVelocityControlSlot, kDriveVelocityIZone, kTalonTimeoutMs);
		
		lMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, kDriveVelocityAllowableError, kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, kDriveVelocityAllowableError, kTalonTimeoutMs);

		
		// Load base lock control gains
		lMotorMaster.config_kF(kBaseLockControlSlot, kDriveBaseLockKf, kTalonTimeoutMs);
		lMotorMaster.config_kP(kBaseLockControlSlot, kDriveBaseLockKp, kTalonTimeoutMs);
		lMotorMaster.config_kI(kBaseLockControlSlot, kDriveBaseLockKi, kTalonTimeoutMs);
		lMotorMaster.config_kD(kBaseLockControlSlot, kDriveBaseLockKd, kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kBaseLockControlSlot, kDriveBaseLockIZone, kTalonTimeoutMs);

		rMotorMaster.config_kF(kBaseLockControlSlot, kDriveBaseLockKf, kTalonTimeoutMs);
		rMotorMaster.config_kP(kBaseLockControlSlot, kDriveBaseLockKp, kTalonTimeoutMs);
		rMotorMaster.config_kI(kBaseLockControlSlot, kDriveBaseLockKi, kTalonTimeoutMs);
		rMotorMaster.config_kD(kBaseLockControlSlot, kDriveBaseLockKd, kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kBaseLockControlSlot, kDriveBaseLockIZone, kTalonTimeoutMs);

		lMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, kDriveBaseLockAllowableError, kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, kDriveBaseLockAllowableError, kTalonTimeoutMs);
		
		lMotorMaster.configOpenloopRamp(kDriveVelocityRampRate, 0);
		rMotorMaster.configOpenloopRamp(kDriveVelocityRampRate, 0);

		/*****************************************************************
		 * Configure Slave Motor Controllers
		 *****************************************************************/
		lMotorSlaves = new ArrayList<BaseMotorController>();	
		rMotorSlaves = new ArrayList<BaseMotorController>();	
		lMotorSlaves.add(new TalonFX(Constants.kLeftSlaveID));
		rMotorSlaves.add(new TalonFX(Constants.kRightSlaveID));			
		
        for (BaseMotorController lMotorSlave : lMotorSlaves) 
        {
    		lMotorSlave.follow(lMotorMaster);	// give slave the TalonID of it's master
    		lMotorSlave.setNeutralMode(NeutralMode.Coast);
			lMotorSlave.setInverted(InvertType.FollowMaster);
        }
        for (BaseMotorController rMotorSlave : rMotorSlaves) 
        {
    		rMotorSlave.follow(rMotorMaster);	// give slave the TalonID of it's master
    		rMotorSlave.setNeutralMode(NeutralMode.Coast);
			rMotorSlave.setInverted(InvertType.FollowMaster);
        }
        
		/*****************************************************************
		 * Set Initial Motor Settings
		 *****************************************************************/
		DriveCommand neutralCmd = DriveCommand.COAST();
		setControlMode(neutralCmd);
		setMotors(neutralCmd);
		setNeutralMode(neutralCmd);
		resetEncoders(neutralCmd);        

	
		/*****************************************************************
		 * Select which Gyro is installed
		 *****************************************************************/
		// select which gyro is installed
		System.out.println("Selected gyro = Pigeon");
		gyro = Pigeon.getInstance();

	}
	
	
	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
		// get status from hardware
		getStatus();
		
		// send new commands to hardware
		sendCommands();
	}

	@Override public void onStop()
	{
		stopMotors();
	}

	private void stopMotors()
	{
		drive.setCommand(DriveCommand.COAST());		// override any incoming commands 
		sendCommands();
	}

	private void getStatus()
	{
		synchronized (driveState)	// lock DriveState until we update it, so that objects reading DriveState don't get partial updates	
		{
			// get Talon control & brake modes (assume right motor is configured identically)
			driveState.setTalonControlMode( lMotorMaster.getControlMode() );
			driveState.setNeutralMode( DriveCommand.getNeutralMode() );
			
			// get encoder values from hardware, set in Drive
			driveState.setLeftDistanceInches(  encoderUnitsToInches( lMotorMaster.getSelectedSensorPosition( kTalonPidIdx ) ));
			driveState.setRightDistanceInches( encoderUnitsToInches( rMotorMaster.getSelectedSensorPosition( kTalonPidIdx ) ));
	
			driveState.setLeftSpeedInchesPerSec(  encoderUnitsPerFrameToInchesPerSecond( lMotorMaster.getSelectedSensorVelocity(  kTalonPidIdx  ) ));
			driveState.setRightSpeedInchesPerSec( encoderUnitsPerFrameToInchesPerSecond( rMotorMaster.getSelectedSensorVelocity(  kTalonPidIdx  ) ));
				
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			driveState.setHeadingDeg( gyro.getHeadingDeg() );
	
			driveState.setMotorCurrent(lMotorMaster.getStatorCurrent(), rMotorMaster.getStatorCurrent() );
			driveState.setMotorPIDError(lMotorMaster.getClosedLoopError( kTalonPidIdx ), rMotorMaster.getClosedLoopError( kTalonPidIdx ) );
	
	        switch (driveState.getTalonControlMode())
	        {
	        	case PercentOutput: 
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	                break;
	
	        	case Position:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorPosition( kTalonPidIdx ), rMotorMaster.getSelectedSensorPosition( kTalonPidIdx ) );
	        		break;
	        		
	        	case Velocity:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorVelocity( kTalonPidIdx ), rMotorMaster.getSelectedSensorVelocity( kTalonPidIdx ) );
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	        		break;
			}
		}
	}
		
	private void sendCommands()
	{
		DriveCommand newCmd = drive.getCommand();
		
		// Watchdog timer  
		double currentTime = Timer.getFPGATimestamp();
		if (currentTime - newCmd.getCommandTime() > kDriveWatchdogTimerThreshold)
		{
			// Halt robot if new command hasn't been sent in a while
			stopMotors();
			return;
		}
				
		synchronized(newCmd)	// lock DriveCommand so no one changes it under us while we are sending the commands
		{
			setControlMode(newCmd);
			setMotors(newCmd);
			setNeutralMode(newCmd);
			resetEncoders(newCmd);
		}
	}
	
	
	private void setControlMode(DriveCommand newCmd)
    {
		ControlMode newMode = newCmd.getTalonControlMode();
		
		if (newMode != driveState.getTalonControlMode())
		{
	        switch (newMode)
	        {
	        	case PercentOutput: 
	                break;
	
	        	case Position:
	    			lMotorMaster.selectProfileSlot(kBaseLockControlSlot, kTalonPidIdx);
	    			rMotorMaster.selectProfileSlot(kBaseLockControlSlot, kTalonPidIdx);

	        		lMotorMaster.set(ControlMode.Position, (double)(lMotorMaster.getSelectedSensorPosition( kTalonPidIdx )) );
	        		rMotorMaster.set(ControlMode.Position, (double)(rMotorMaster.getSelectedSensorPosition( kTalonPidIdx )) );
	        		break;
	        		
	        	case Velocity:
	        		lMotorMaster.selectProfileSlot(kVelocityControlSlot, kTalonPidIdx);
	        		rMotorMaster.selectProfileSlot(kVelocityControlSlot, kTalonPidIdx);
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		break;
	        }
		}
	}
	
	
	
	private void setNeutralMode(DriveCommand newCmd)
	{
		NeutralMode newNeutral = DriveCommand.getNeutralMode();
		setNeutralMode(newNeutral);
	}
	
	
	private void setNeutralMode(NeutralMode newNeutral) 
	{
		if (newNeutral != driveState.getNeutralMode()) 
		{
			lMotorMaster.setNeutralMode(newNeutral);
			rMotorMaster.setNeutralMode(newNeutral);
			for (BaseMotorController lMotorSlave : lMotorSlaves)
				lMotorSlave.setNeutralMode(newNeutral);
			for (BaseMotorController rMotorSlave : rMotorSlaves)
				rMotorSlave.setNeutralMode(newNeutral);
		}
	}
	
	
		
	private void setMotors(DriveCommand newCmd)
    {
		double lMotorCtrl = newCmd.getLeftMotor();
		double rMotorCtrl = newCmd.getRightMotor();
		
        switch (newCmd.getTalonControlMode())	// assuming new mode is already configured
        {
        	case PercentOutput:
        		// DriveCommand given in range +/-1, with 1 representing full throttle
        		lMotorMaster.set(ControlMode.PercentOutput, lMotorCtrl);
        		rMotorMaster.set(ControlMode.PercentOutput, rMotorCtrl);
        		break;

        	case Position:
        		// no changes to position set in setControlMode()
        		break;
        		
        	case Velocity:
        		// DriveCommand given in inches/sec
        		// Talon SRX needs RPM in closed-loop mode.
        		// convert inches/sec to encoder edges per 100ms
           		lMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderUnitsPerFrame(lMotorCtrl)); 
        		rMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderUnitsPerFrame(rMotorCtrl));
        		break;
        		
        	case Disabled:
        	default:
        		lMotorMaster.set(ControlMode.Disabled, 0);
        		rMotorMaster.set(ControlMode.Disabled, 0);
        		break;
        }
	}

	// Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToInches(double _encoderPosition) { return _encoderPosition / kFalconEncoderUnitsPerRev / kDriveGearRatio * kDriveWheelCircumInches; }
	public static double inchesToEncoderUnits(double _inches) { return _inches / kDriveWheelCircumInches * kFalconEncoderUnitsPerRev * kDriveGearRatio; }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToInchesPerSecond(double _encoderEdgesPerFrame) { return encoderUnitsToInches(_encoderEdgesPerFrame) / kFalconEncoderStatusFramePeriod; }
	public static int inchesPerSecondToEncoderUnitsPerFrame(double _inchesPerSecond) { return (int)(inchesToEncoderUnits(_inchesPerSecond) * kFalconEncoderStatusFramePeriod); }

	
	

	private void resetEncoders(DriveCommand newCmd)
	{
		if (newCmd.getResetEncoders())
		{
			lMotorMaster.setSelectedSensorPosition(0.0);
			rMotorMaster.setSelectedSensorPosition(0.0);
			gyro.zeroSensor();  
			// calibration to desired initial pose is done in RobotState.reset() called from Robot.autonomousInit()  
		}
	}	
};
