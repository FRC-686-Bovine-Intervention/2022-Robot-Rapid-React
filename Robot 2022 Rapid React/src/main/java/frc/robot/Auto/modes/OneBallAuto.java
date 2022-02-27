package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SetIntake;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment.Options;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Intake.IntakeState;

public class OneBallAuto extends AutoMode{
    public OneBallAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        Path reversePath = new Path();
        Vector2d initialPose = new Vector2d(0,0);
        Vector2d outsideTarmac = new Vector2d(-120,0);
        Options driveOptions = new Options(24,36,36,false);

        reversePath.add(new Waypoint(initialPose, driveOptions));
        reversePath.add(new Waypoint(outsideTarmac, driveOptions));
        reversePath.setReverseDirection();
        runAction(new SetIntake(IntakeState.OUTTAKE));
        runAction(new WaitAction(0.5));
        runAction(new SetIntake(IntakeState.DEFENSE));
        runAction(new PathFollowerAction(reversePath));
    }
}
