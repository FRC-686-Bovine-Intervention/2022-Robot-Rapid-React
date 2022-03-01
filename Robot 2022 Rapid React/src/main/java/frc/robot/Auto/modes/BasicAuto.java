package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.TurnToAngle;

public class BasicAuto extends AutoMode{
    public BasicAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new TurnToAngle(180));

        //runAction(new SetIntake(Intake.IntakeState.INTAKE));
        //runAction(new SetIntake(Intake.IntakeState.OUTTAKE));
    }
}
