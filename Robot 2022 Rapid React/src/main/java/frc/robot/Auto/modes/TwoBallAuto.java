package frc.robot.auto.modes;

import java.util.Arrays;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake.IntakeState;

public class TwoBallAuto extends AutoMode{
    public TwoBallAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {

        double maxSpeed = 36;
        double accel = 36;
        double lookaheadDist = 24;
        boolean visionEnabled = false;
        Options driveOptions = new Options(maxSpeed, accel, lookaheadDist, visionEnabled);

        // Robot starts with front bumper aligned with tarmac tape, with bumper corner at apex corner of tarmax
        
        // path1: drive forward to collect one more ball
        Vector2d ourBall4IntakePos = FieldDimensions.ourBall4.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(-35.35))); 
        Path path1 = new Path();
        path1.add(new Waypoint(FieldDimensions.ourBall4StartPose.getPosition(), driveOptions));    // start at tarmac tape
        path1.add(new Waypoint(ourBall4IntakePos, driveOptions));   // stop when front bumpers reach center of ball position

        // path2: after turning around, drive from intake position to the left fender, passing through
        // a spot in front of the fender to get a good angle
        Vector2d lFenderApproachPos = FieldDimensions.lFenderCenter.sub(Vector2d.magnitudeAngle(48.0 ,FieldDimensions.lFenderApproachAngleRad));
        Path path2 = new Path();
        path2.add(new Waypoint(ourBall4IntakePos, driveOptions));
        path2.add(new Waypoint(lFenderApproachPos, driveOptions));
        path2.add(new Waypoint(FieldDimensions.lFenderShotPose.getPosition(), driveOptions));

        // path3: back up a short distance after shooting
        Vector2d lFenderBackupPos = FieldDimensions.lFenderCenter.sub(Vector2d.magnitudeAngle(24.0 ,FieldDimensions.lFenderApproachAngleRad));
        Path path3 = new Path();
        path3.add(new Waypoint(FieldDimensions.lFenderShotPose.getPosition(), driveOptions));
        path3.add(new Waypoint(lFenderBackupPos, driveOptions));
        path3.setReverseDirection();

        // path4: after backing up and turning towards theirBall2, intake theirBall2, and shoot it into the hangar
        Vector2d theirBall2IntakePos = FieldDimensions.theirBall2.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(-35.35)));
        Vector2d outtakePos = theirBall2IntakePos.add(new Vector2d(24.0, 0.0));
        Path path4 = new Path();
        path4.add(new Waypoint(lFenderBackupPos, driveOptions));
        path4.add(new Waypoint(theirBall2IntakePos, driveOptions));
        path4.add(new Waypoint(outtakePos, driveOptions));

        // path5: after turning around, get close to ourBall5 on the other side of the field
        Path path5 = new Path();
        path5.add(new Waypoint(outtakePos, driveOptions));
        path5.add(new Waypoint(FieldDimensions.ourBall5EndPose.getPosition(), driveOptions));


        //================================================================
        // TWO BALL AUTO
        // (plus yeeting other teams ball into hangar)
        //================================================================
        runAction(new WaitAction(0.0));     // TODO: use programmable delay from Shuffleboard

        // drive forward, intake ball
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path1))));

        // turn around towards fender, then drive to fender and shoot
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new TurnToAngleAction(180 - 35.25))));
        runAction(new PathFollowerAction(path2));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(0.3));

        // back up a short distance, turn towards theirBall2
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path3))));
        runAction(new TurnToAngleAction(-54.75));
        // drive to theirBall2, intake, drive a little longer then outtake into hangar
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path4))));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE_GROUND));
        runAction(new WaitAction(0.3));

        // turn around, head towards ourBall5
        runAction(new TurnToAngleAction(180));
        runAction(new PathFollowerAction(path5));
    }
}
