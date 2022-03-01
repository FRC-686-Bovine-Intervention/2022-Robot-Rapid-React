package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.TurnToAngleAction;

public class TurnToAngleAuto extends AutoMode{
    public TurnToAngleAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new TurnToAngleAction(90));
    }
}
