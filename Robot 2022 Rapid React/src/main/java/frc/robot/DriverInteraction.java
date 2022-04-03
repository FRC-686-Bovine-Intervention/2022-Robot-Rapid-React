package frc.robot; 
 
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveCommand.DriveControlMode;
import frc.robot.controls.Controls;
import frc.robot.controls.Controls.ButtonControlEnum;
import frc.robot.controls.Controls.JoystickEnum;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.vision.VisionDriveAssistant; 
 
public class DriverInteraction { 
    private static DriverInteraction instance; 
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;} 
 
    private static final double kClimberMaxPercent = 1; 
    private static final double kClimbingDriveSlowdown = 0.3; 
 
    Drive drive; 
    Controls controls; 
    Climber climber; 
    Intake intake; 
 
    private DriverInteraction() 
    { 
        controls = Controls.getInstance(); 
        drive = Drive.getInstance(); 
        intake = Intake.getInstance(); 
        climber = Climber.getInstance(); 
        /*for (Subsystem s : SubsystemManager.getInstance().subsystems) 
        { 
            if (s instanceof Drive)         {drive =        (Drive)s;} 
            if (s instanceof Climber)       {climber =      (Climber)s;} 
            if (s instanceof Intake)        {intake =       (Intake)s;} 
        }*/ 
    } 
 
    private RisingEdgeDetector climberNextEdgeDetector = new RisingEdgeDetector(); 
    private RisingEdgeDetector climberPrevEdgeDetector = new RisingEdgeDetector(); 
     
    public void init() {
        climber.resetState();
    }
 
    public void run() 
    { 
        DriveCommand driveCommand = DriveCommand.COAST();
        climberNextEdgeDetector.update(controls.getButton(ButtonControlEnum.CLIMBER_NEXT_STATE)); 
        climberPrevEdgeDetector.update(controls.getButton(ButtonControlEnum.CLIMBER_PREV_STATE)); 
        if(climberNextEdgeDetector.get())
        {
            climber.nextState();
        }
        if (climberPrevEdgeDetector.get())
        {
            climber.prevState();
        }
        if (controls.getButton(ButtonControlEnum.CLIMBER_RESET_STATE))
        {
            climber.resetState();
        }
        switch(climber.climberStatus)
        {
            case DEFENSE:
            case CALIBRATING:
                Shuffleboard.selectTab("Intake");
                if (controls.getButton(ButtonControlEnum.INTAKE) && controls.getButton(ButtonControlEnum.OUTTAKE)) 
                { 
                    intake.setState(IntakeState.OUTTAKE_GROUND); 
                } 
                else if (controls.getButton(ButtonControlEnum.INTAKE)) 
                { 
                    intake.setState(IntakeState.INTAKE); 
                } 
                else if (controls.getButton(ButtonControlEnum.OUTTAKE)) 
                { 
                    intake.setState(IntakeState.OUTTAKE); 
                } 
                else 
                { 
                    intake.setState(IntakeState.DEFENSE); 
                }
                driveCommand = VisionDriveAssistant.getInstance().assist(controls.getDriveCommand(), controls.getButton(ButtonControlEnum.VISION_ASSIST) && intake.intakeStatus == IntakeState.INTAKE);
            break;
            case LOW_BAR:
                driveCommand = controls.getDriveCommand();
                Shuffleboard.selectTab("Climber");
            break;
            case SLOW_DRIVE:
                driveCommand = new DriveCommand(DriveControlMode.OPEN_LOOP, controls.getDriveCommand().getLeftMotor()*kClimbingDriveSlowdown, controls.getDriveCommand().getRightMotor()*kClimbingDriveSlowdown, NeutralMode.Coast);
                Shuffleboard.selectTab("Climber");
            break;
            default:
                Shuffleboard.selectTab("Climber");
                climber.setTargetPos(controls.getAxis(JoystickEnum.THRUSTMASTER).y*kClimberMaxPercent);
            break;
        }
        drive.setOpenLoop(driveCommand);
    } 
} 
