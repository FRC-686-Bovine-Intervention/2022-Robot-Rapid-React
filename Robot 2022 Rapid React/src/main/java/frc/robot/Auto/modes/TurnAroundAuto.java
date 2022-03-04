package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveDistanceAction;
import frc.robot.auto.actions.TurnToAngleAction;

public class TurnAroundAuto extends AutoMode{
    public TurnAroundAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveDistanceAction(24, 24));
        runAction(new TurnToAngleAction(180));
        runAction(new DriveDistanceAction(-24, -24));
        
        runAction(new DriveDistanceAction(24, 24));
        runAction(new TurnToAngleAction(0));
        runAction(new DriveDistanceAction(-24, -24));
    }
}
