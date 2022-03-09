package frc.robot;

import frc.robot.controls.Controls;
import frc.robot.controls.Controls.ButtonControlEnum;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class DriverInteraction {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    Drive drive;
    Controls controls;
    Climber climber;
    Intake intake;

    private DriverInteraction()
    {
        controls = Controls.getInstance();
        drive = Drive.getInstance();
        intake = Intake.getInstance();
        /*for (Subsystem s : SubsystemManager.getInstance().subsystems)
        {
            if (s instanceof Drive)         {drive =        (Drive)s;}
            if (s instanceof Climber)       {climber =      (Climber)s;}
            if (s instanceof Intake)        {intake =       (Intake)s;}
        }*/
    }

    private boolean driving = true;

    public void run()
    {
        driving = !controls.getButton(Controls.ButtonControlEnum.CLIMBERNEXTSTAGE);
        if (driving) 
        {
            drive.setOpenLoop(controls.getDriveCommand());
            //Climber.getInstance().setTargetPos(0);
        }
        //else Climber.getInstance().setTargetPos(controls.getAxis(JoystickEnum.THRUSTMASTER).y*-1);
        if (controls.getButton(Controls.ButtonControlEnum.INTAKE) && controls.getButton(ButtonControlEnum.OUTTAKE))
        {
            intake.changeState(IntakeState.OUTTAKE_GROUND);
        }
        else if (controls.getButton(Controls.ButtonControlEnum.INTAKE))
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
