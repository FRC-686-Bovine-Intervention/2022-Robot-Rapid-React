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
import frc.robot.command_status.RobotState;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment.Options;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Intake.IntakeState;

public class TwoBallAuto extends AutoMode{
    public TwoBallAuto(){
        System.out.println("ourBall4StartPose: " + FieldDimensions.ourBall4StartPose);
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        double maxSpeed = 24;//36;
        double accel = 24;//36;
        double lookaheadDist = 24;
        boolean visionEnabled = false;
        Options driveOptions = new Options(maxSpeed, accel, lookaheadDist, visionEnabled);

        double shotTime = 0.3;

        // Robot starts with front bumper aligned with tarmac tape, with bumper corner at apex corner of tarmax
        
        // path1: drive forward to collect one more ball
        Vector2d ourBall4StartPos = FieldDimensions.ourBall4StartPose.getPosition();
        Vector2d ourBall4IntakePos = FieldDimensions.ourBall4.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(-35.35))); 
        Path path1 = new Path();
        path1.add(new Waypoint(ourBall4StartPos, driveOptions));    // start at tarmac tape
        path1.add(new Waypoint(ourBall4IntakePos, driveOptions));   // stop when front bumpers reach center of ball position

        // path2: after turning around, drive from intake position to the left fender, passing through
        // a spot in front of the fender to get a good angle
        Vector2d lFenderApproachPos = FieldDimensions.lFenderCenter.sub(Vector2d.magnitudeAngle(48.0 ,FieldDimensions.lFenderApproachAngleRad));
        Vector2d lFenderShotPos = FieldDimensions.lFenderShotPose.getPosition();
        Path path2 = new Path();
        path2.add(new Waypoint(ourBall4IntakePos, driveOptions));
        path2.add(new Waypoint(lFenderApproachPos, driveOptions));
        path2.add(new Waypoint(lFenderShotPos, driveOptions));

        // path3: back up a short distance after shooting
        Vector2d lFenderBackupPos = lFenderShotPos.sub(Vector2d.magnitudeAngle(24.0 ,FieldDimensions.lFenderApproachAngleRad));
        Path path3 = new Path();
        path3.add(new Waypoint(lFenderShotPos, driveOptions));
        path3.add(new Waypoint(lFenderBackupPos, driveOptions));
        path3.setReverseDirection();

        // path4: after backing up and turning towards theirBall3, intake theirBall3, and shoot it into the hangar
        Vector2d theirBall3IntakePos = FieldDimensions.theirBall3.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(-35.35)));
        Vector2d outtakePos = theirBall3IntakePos.add(new Vector2d(12.0, 0.0));
        Path path4 = new Path();
        path4.add(new Waypoint(lFenderBackupPos, driveOptions));
        path4.add(new Waypoint(theirBall3IntakePos, driveOptions));
        path4.add(new Waypoint(outtakePos, driveOptions));

        // path5: after turning around, get close to ourBall5 on the other side of the field
        Vector2d ourBall5EndPos = FieldDimensions.theirBall3.add(new Vector2d(0, -12));
        Path path5 = new Path();
        path5.add(new Waypoint(outtakePos, driveOptions));
        path5.add(new Waypoint(ourBall5EndPos, driveOptions));


        //================================================================
        // TWO BALL AUTO
        // (plus yeeting other teams ball into hangar)
        //================================================================
        // set initial pose
        RobotState.getInstance().reset(FieldDimensions.ourBall4StartPose);

        runAction(new WaitAction(0.0));     // TODO: use programmable delay from Shuffleboard

        // drive forward, intake ball
        // runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path1))));
        runAction(new SetIntakeAction(IntakeState.INTAKE));
        runAction(new PathFollowerAction(path1));

        // turn around towards fender, then drive to fender and shoot
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new TurnToAngleAction(180 - 35.25))));
        runAction(new PathFollowerAction(path2));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(shotTime));

        // back up a short distance, turn towards theirBall3
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path3))));
        runAction(new TurnToAngleAction(-75));
        // drive to theirBall3, intake, drive a little longer then outtake into hangar
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path4))));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE_GROUND));
        runAction(new WaitAction(shotTime));

        // TODO?  Could hide ball behind truss structure if we have enough time

        // turn around, head towards ourBall5
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new TurnToAngleAction(180.0))));
        // runAction(new PathFollowerAction(path5));
    }
}
