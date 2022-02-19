package frc.robot;

import frc.robot.Controls.Controls;
import frc.robot.Subsystems.Subsystems.Climber;
import frc.robot.Subsystems.Subsystems.Drive;
import frc.robot.Subsystems.Subsystems.Drivetrain;
import frc.robot.Subsystems.Subsystems.Intake;

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
        /*if (controls.getButton(Controls.ButtonControlEnum.INTAKE))
        {
            Intake.getInstance().changeState(IntakeState.INTAKE);
        }
        else if (controls.getButton(Controls.ButtonControlEnum.OUTTAKE))
        {
            Intake.getInstance().changeState(IntakeState.OUTTAKE);
        }
        else
        {
            Intake.getInstance().changeState(IntakeState.DEFENSE);
        }*/
    }
}
