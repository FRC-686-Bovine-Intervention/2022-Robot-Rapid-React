package frc.robot.Subsystems.Subsystems;

import frc.robot.Controls.Controls;
import frc.robot.Subsystems.Subsystem;
import frc.robot.Subsystems.SubsystemManager;

public class DriverInteraction {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    Controls controls;
    Drivetrain drivetrain;
    Climber climber;
    Intake intake;

    public DriverInteraction()
    {
        controls = Controls.getInstance();
        for (Subsystem s : SubsystemManager.getInstance().subsystems)
        {
            if (s instanceof Drivetrain)    {drivetrain =   (Drivetrain)s;}
            if (s instanceof Climber)       {climber =      (Climber)s;}
            if (s instanceof Intake)        {intake =       (Intake)s;}
        }
    }

    public void run()
    {
        drivetrain.setAxis(controls.getAxis(Controls.JoystickEnum.THRUSTMASTER));
    }
}
