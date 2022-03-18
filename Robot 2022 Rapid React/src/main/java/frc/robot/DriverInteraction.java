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
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Intake.IntakeState; 
 
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
 
    private RisingEdgeDetector climbEdgeDetector = new RisingEdgeDetector(); 
     
    public void init() {
        climber.setState(ClimberState.DEFENSE);
    }
 
    public void run() 
    { 
        climbEdgeDetector.update(controls.getButton(ButtonControlEnum.CLIMBERNEXTSTAGE)); 
        if(climbEdgeDetector.get())
        {
            climber.nextState();
        }
        if (controls.getButton(ButtonControlEnum.CLIMBERPREVSTAGE))
        {
            climber.setState(ClimberState.DEFENSE);
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
                drive.setOpenLoop(controls.getDriveCommand());
            break;
            case LOW_BAR:
                drive.setOpenLoop(controls.getDriveCommand());
                Shuffleboard.selectTab("Climber");
            break;
            case SLOW_DRIVE:
                drive.setOpenLoop(new DriveCommand(DriveControlMode.OPEN_LOOP, controls.getDriveCommand().getLeftMotor()*kClimbingDriveSlowdown, controls.getDriveCommand().getRightMotor()*kClimbingDriveSlowdown, NeutralMode.Coast));
                Shuffleboard.selectTab("Climber");
            break;
            default:
                Shuffleboard.selectTab("Climber");
                climber.setTargetPos(controls.getAxis(JoystickEnum.THRUSTMASTER).y*kClimberMaxPercent);
            break;
        }
    } 
} 
