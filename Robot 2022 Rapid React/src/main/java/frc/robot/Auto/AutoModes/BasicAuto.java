package frc.robot.Auto.AutoModes;

import frc.robot.Auto.AutoModeEndedException;
import frc.robot.Auto.Actions.DriveStraight;
import frc.robot.Auto.Actions.SetIntake;
import frc.robot.Subsystems.Subsystems.Intake;

public class BasicAuto extends AutoMode{
    public BasicAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveStraight(36));

        runAction(new SetIntake(Intake.IntakeState.INTAKE));
        runAction(new SetIntake(Intake.IntakeState.OUTTAKE));
    }
}
