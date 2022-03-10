package frc.robot; 
 
import com.ctre.phoenix.motorcontrol.NeutralMode;

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
 
public class DriverInteraction { 
    private static DriverInteraction instance; 
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;} 
 
    private static final double kClimberMaxPercent = 1; 
    private static final double kClimbingDriveSlowdown = 0.3; 
    private static final double kIntakeMaxPercent = 0.3; 
 
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
 
    private enum SubsystemControl 
    { 
        DRIVETOCLIMB, 
        CLIMBTODRIVE, 
        DRIVETOINTAKE, 
        CLIMBTOINTAKE, 
        INTAKE 
    } 
 
    private RisingEdgeDetector climbEdgeDetector = new RisingEdgeDetector(); 
    private SubsystemControl subsystemControl = SubsystemControl.DRIVETOCLIMB; 
     
    public void init() 
    { 
        subsystemControl = SubsystemControl.DRIVETOCLIMB; 
    } 
 
    public void run() 
    { 
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
        climbEdgeDetector.update(controls.getButton(ButtonControlEnum.CLIMBERNEXTSTAGE)); 
        if (climbEdgeDetector.get()) 
        { 
            switch(subsystemControl) 
            { 
                case DRIVETOCLIMB:  subsystemControl = SubsystemControl.CLIMBTODRIVE;   break; 
                case CLIMBTODRIVE:  subsystemControl = SubsystemControl.DRIVETOINTAKE;  break; 
                case DRIVETOINTAKE: subsystemControl = SubsystemControl.CLIMBTOINTAKE;  break; 
                case CLIMBTOINTAKE: subsystemControl = SubsystemControl.INTAKE;         break; 
                case INTAKE:        subsystemControl = SubsystemControl.CLIMBTOINTAKE;  break; 
            } 
        } 
        switch(subsystemControl) 
        { 
            case DRIVETOCLIMB: 
                drive.setOpenLoop(controls.getDriveCommand()); 
            break; 
            case DRIVETOINTAKE: 
                drive.setOpenLoop(new DriveCommand(DriveControlMode.OPEN_LOOP, controls.getDriveCommand().getLeftMotor()*kClimbingDriveSlowdown, controls.getDriveCommand().getRightMotor()*kClimbingDriveSlowdown, NeutralMode.Coast)); 
            break; 
            case CLIMBTOINTAKE: 
                intake.setState(IntakeState.CLIMBING); 
            case CLIMBTODRIVE: 
                climber.setTargetPos(controls.getAxis(JoystickEnum.THRUSTMASTER).y*kClimberMaxPercent); 
            break; 
            case INTAKE: 
                intake.setState(IntakeState.CLIMBING); 
                intake.setClimbingPower(controls.getAxis(JoystickEnum.THRUSTMASTER).y*kIntakeMaxPercent); 
            break; 
        } 
    } 
} 
