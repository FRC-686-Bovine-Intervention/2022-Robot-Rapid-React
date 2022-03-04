package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetIntakeAction;
import frc.robot.subsystems.Intake;

public class BasicAuto extends AutoMode{
    public BasicAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetIntakeAction(Intake.IntakeState.INTAKE));
        runAction(new SetIntakeAction(Intake.IntakeState.OUTTAKE));
    }
}
