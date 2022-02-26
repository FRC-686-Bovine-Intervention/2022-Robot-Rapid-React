package frc.robot;

import frc.robot.controls.Controls;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

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

    public void run()
    {
        Drive.getInstance().setOpenLoop(controls.getDriveCommand());
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
