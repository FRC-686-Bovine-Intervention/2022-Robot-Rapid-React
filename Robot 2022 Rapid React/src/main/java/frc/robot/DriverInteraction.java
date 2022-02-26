package frc.robot;

import frc.robot.Controls.Controls;
import frc.robot.Controls.Controls.ButtonControlEnum;
import frc.robot.Controls.Controls.JoystickEnum;
import frc.robot.Subsystems.Subsystems.Climber;
import frc.robot.Subsystems.Subsystems.Drive;
import frc.robot.Subsystems.Subsystems.Drivetrain;
import frc.robot.Subsystems.Subsystems.Intake;
import frc.robot.Subsystems.Subsystems.Intake.IntakeState;

public class DriverInteraction {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    Controls controls;
    Drivetrain drivetrain;
    Climber climber;
    Intake intake;

    private DriverInteraction()
    {
        controls = Controls.getInstance();
        intake = Intake.getInstance();
        /*for (Subsystem s : SubsystemManager.getInstance().subsystems)
        {
            if (s instanceof Drivetrain)    {drivetrain =   (Drivetrain)s;}
            if (s instanceof Climber)       {climber =      (Climber)s;}
            if (s instanceof Intake)        {intake =       (Intake)s;}
        }*/
    }

    private boolean driving = true;

    public void run()
    {
        driving = !controls.getButton(Controls.ButtonControlEnum.CLIMBERNEXTSTAGE);
        if (driving) Drive.getInstance().setOpenLoop(controls.getDriveCommand());
        else Climber.getInstance().setTargetPos(controls.getAxis(JoystickEnum.THRUSTMASTER).y*-0.6);
        if (controls.getButton(Controls.ButtonControlEnum.INTAKE))
        {
            intake.changeState(IntakeState.INTAKE);
        }
        else if (controls.getButton(Controls.ButtonControlEnum.OUTTAKE))
        {
            intake.changeState(IntakeState.OUTTAKE);
        }
        else
        {
            intake.changeState(IntakeState.DEFENSE);
        }
    }
}
