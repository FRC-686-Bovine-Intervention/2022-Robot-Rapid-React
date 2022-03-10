package frc.robot.auto.modes;

import java.util.Arrays;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.auto.AutoManager;
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
        initialPose = FieldDimensions.twoBallAutoStartingPose;
        System.out.println("twoBallAutoStartingPose: " + FieldDimensions.twoBallAutoStartingPose);
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        double maxSpeed = 24;//36;
        double accel = 24;//36;
        double lookaheadDist = 24;
        boolean visionEnabled = false;
        Options driveOptions = new Options(maxSpeed, accel, lookaheadDist, visionEnabled);

        double shotTime = 0.5;

        // Robot starts with front bumper aligned with tarmac tape, with bumper corner at apex corner of tarmax
        
        // path1: drive forward to collect one more ball
        Vector2d initialPos = initialPose.getPosition();
        Vector2d ball2of2IntakePos = FieldDimensions.ball2of2.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(-20.0))); 
        Path path1 = new Path();
        path1.add(new Waypoint(initialPos, driveOptions));    // start at tarmac tape
        path1.add(new Waypoint(ball2of2IntakePos, driveOptions));   // stop when front bumpers reach center of ball position

        // path2: after turning around, drive from intake position to the left fender, passing through
        // a spot in front of the fender to get a good angle
        Path path2 = new Path();
        path2.add(new Waypoint(ball2of2IntakePos, driveOptions));
        path2.add(new Waypoint(FieldDimensions.fenderApproachPos, driveOptions));
        path2.add(new Waypoint(FieldDimensions.fenderShotPos, driveOptions));

        // path3: back up a short distance after shooting
        Path path3 = new Path();
        path3.add(new Waypoint(FieldDimensions.fenderShotPos, driveOptions));
        path3.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path3.setReverseDirection();

        // path4: after backing up and turning towards theirBall, intake theirBall, and shoot it into the hangar
        double theirBallHeadingRad = FieldDimensions.theirBall.sub(FieldDimensions.fenderBackupPos).angle();
        Vector2d theirBallIntakePos = FieldDimensions.theirBall.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, theirBallHeadingRad));
        Path path4 = new Path();
        path4.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path4.add(new Waypoint(theirBallIntakePos, driveOptions));
        
        double hangarShotAngleDeg = Units.radiansToDegrees(FieldDimensions.hangarShotPos.sub(theirBallIntakePos).angle());

        //================================================================
        // TWO BALL AUTO
        // (plus yeeting other teams ball into hangar)
        //================================================================
        RobotState.getInstance().reset(initialPose);

        runAction(new WaitAction(AutoManager.autoInitialDelaySec)); 

        // drive forward, intake ball
        runAction(new SetIntakeAction(IntakeState.INTAKE));
        runAction(new PathFollowerAction(path1));

        // turn around towards fender, then drive to fender and shoot
        double backTowardsFenderHeadingDeg = Units.radiansToDegrees(FieldDimensions.fenderApproachPos.sub(ball2of2IntakePos).angle());
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new TurnToAngleAction(backTowardsFenderHeadingDeg))));
        runAction(new PathFollowerAction(path2));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(shotTime));
        
        // back up a short distance
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path3))));
        
        // turn and drive  to theirBall, intake
        double theirBallHeadingDeg = Units.radiansToDegrees(FieldDimensions.theirBall.sub(FieldDimensions.fenderBackupPos).angle());
        runAction(new TurnToAngleAction(theirBallHeadingDeg));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path4))));

        // turn towards hangar and shoot their ball into it
        runAction(new TurnToAngleAction(hangarShotAngleDeg));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE_GROUND));        
        runAction(new WaitAction(shotTime));
        
        // turn around, head towards ourBallOtherSide
        double ourBallOtherSideHeadingDeg = Units.radiansToDegrees(FieldDimensions.twoBallAutoFinalTarget.sub(theirBallIntakePos).angle());
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new TurnToAngleAction(ourBallOtherSideHeadingDeg))));
    }
}
