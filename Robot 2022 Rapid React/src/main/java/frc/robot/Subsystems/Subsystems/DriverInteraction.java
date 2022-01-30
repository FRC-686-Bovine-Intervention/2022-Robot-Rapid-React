package frc.robot.Subsystems.Subsystems;

import frc.robot.Controls.Controls;
import frc.robot.Controls.Controls.ButtonControlEnum;
import frc.robot.Subsystems.Subsystem;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.Subsystems.Climber.ClimberState;

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
        if (climber.getClimberStatus() == ClimberState.DEFENCE || climber.ClimberStatusHistory.get(climber.ClimberStatusHistory.size()-2) == ClimberState.DEFENCE)
        {
            drivetrain.setAxis(controls.getAxis(Controls.JoystickEnum.THRUSTMASTER));
        }
        else if (climber.getClimberStatus() == ClimberState.EXTEND && climber.readyForNextState)
        {
            //put driver pivot arm control here
        }
        if (controls.getButton(ButtonControlEnum.CLIMBERFORCESTATE) || (controls.getButton(ButtonControlEnum.CLIMBERNEXTSTAGE) && climber.readyForNextState)) {climber.nextState();}
        if (controls.getButton(ButtonControlEnum.CLIMBERPREVSTAGE)) {climber.prevState();}
    }
}
