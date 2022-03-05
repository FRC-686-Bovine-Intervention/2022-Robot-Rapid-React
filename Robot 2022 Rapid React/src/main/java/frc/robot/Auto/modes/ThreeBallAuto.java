package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SetIntakeAction;
import frc.robot.auto.actions.TurnToAngleAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment.Options;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Intake.IntakeState;

public class ThreeBallAuto extends AutoMode{
    public ThreeBallAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        Path path1 = new Path();
        Path path2 = new Path();
        Path path3 = new Path();
        Path path4 = new Path();
        Path path5 = new Path();
        System.out.println(FieldDimensions.rFenderStartPose);
        Vector2d initPos = FieldDimensions.rFenderStartPose.getPosition();
        Vector2d axisOfEvil = Vector2d.magnitudeAngle(72+37, FieldDimensions.rFenderNormalAngle);
        Vector2d bBall1 = axisOfEvil.add(new Vector2d(-48,-24));
        Vector2d lBall1 = FieldDimensions.ourBall1.add(new Vector2d(-48,0));
        Vector2d dBall1 = FieldDimensions.ourBall1.add(new Vector2d(0,-18));
        Vector2d rBall1 = FieldDimensions.ourBall1.add(new Vector2d(36,0));
        Vector2d ball2 = FieldDimensions.ourBall2;
        Vector2d finalPos = FieldDimensions.theirBall1.add(new Vector2d(0,-24));
        Options driveOptions = new Options(36,36,24,false);

        path1.add(new Waypoint(initPos, driveOptions));
        path1.add(new Waypoint(axisOfEvil, driveOptions));
        path1.add(new Waypoint(bBall1, driveOptions));
        path1.setReverseDirection();
        
        path2.add(new Waypoint(bBall1, driveOptions));
        path2.add(new Waypoint(lBall1, driveOptions));
        path2.add(new Waypoint(dBall1, driveOptions));
        path2.add(new Waypoint(rBall1, driveOptions));
        path2.add(new Waypoint(ball2, driveOptions));
        
        path3.add(new Waypoint(ball2, driveOptions));
        path3.add(new Waypoint(axisOfEvil, driveOptions));
        path3.add(new Waypoint(initPos, driveOptions));

        path4.add(new Waypoint(initPos, driveOptions));
        path4.add(new Waypoint(axisOfEvil, driveOptions));
        path4.setReverseDirection();
        
        path5.add(new Waypoint(axisOfEvil, driveOptions));
        path5.add(new Waypoint(finalPos, driveOptions));

        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(0.3));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path1))));
        runAction(new WaitAction(0.1));
        runAction(new PathFollowerAction(path2));
        // runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path3))));
        // runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        // runAction(new WaitAction(0.3));
        // runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path4))));
        // runAction(new PathFollowerAction(path5));
    }
}
