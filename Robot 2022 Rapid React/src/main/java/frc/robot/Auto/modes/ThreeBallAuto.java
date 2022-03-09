package frc.robot.auto.modes;

import java.util.Arrays;
import java.util.concurrent.RejectedExecutionHandler;

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

public class ThreeBallAuto extends AutoMode{
    public ThreeBallAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {

        double maxSpeed = 24;//36;
        double accel = 24;//36;
        double lookaheadDist = 24;
        boolean visionEnabled = false;
        Options driveOptions = new Options(maxSpeed, accel, lookaheadDist, visionEnabled);

        double shotTime = 0.3;

       // path1: after initial shot, backup so we can turn around
        Path path1 = new Path();
        path1.add(new Waypoint(FieldDimensions.fenderShotPos, driveOptions));
        path1.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path1.setReverseDirection();
        
        // path2: drive to ball 1
        double ball1ApproachHeadingRad = FieldDimensions.ourBall1.sub(FieldDimensions.fenderShotPos).angle();
        Vector2d ourBall1IntakePos = FieldDimensions.ourBall1.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, ball1ApproachHeadingRad)); 
        Path path2 = new Path();
        path2.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path2.add(new Waypoint(ourBall1IntakePos, driveOptions));

        // path3: drive to ball 2
        double ball2ApproachHeadingRad = FieldDimensions.ourBall2.sub(ourBall1IntakePos).angle();
        Vector2d ourBall2IntakePos = FieldDimensions.ourBall2.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, ball2ApproachHeadingRad)); 
        Path path3 = new Path();
        path3.add(new Waypoint(ourBall1IntakePos, driveOptions));
        path3.add(new Waypoint(ourBall2IntakePos, driveOptions));

        // path4: go back to the fender and shoot
        double returnAngleRad = FieldDimensions.fenderApproachPos.sub(ourBall2IntakePos).angle();
        Path path4 = new Path();
        path4.add(new Waypoint(ourBall2IntakePos, driveOptions));
        path4.add(new Waypoint(FieldDimensions.fenderApproachPos, driveOptions));
        path4.add(new Waypoint(FieldDimensions.fenderShotPos, driveOptions));

        // path5 backup
        Path path5 = new Path();
        path5.add(new Waypoint(FieldDimensions.fenderShotPos, driveOptions));
        path5.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path5.setReverseDirection();
        
        // path6: head towards ourBall6
        Vector2d finalPos = FieldDimensions.fenderBackupPos.add(new Vector2d(24,0));
        Path path6 = new Path();
        path6.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path6.add(new Waypoint(finalPos, driveOptions));


        //================================================================
        // THREE BALL AUTO
        //================================================================
        
        // set initial pose
        RobotState.getInstance().reset(FieldDimensions.threeBallAutoStartingPose);

        runAction(new WaitAction(0.0));     // TODO: use programmable delay from Shuffleboard

        // shoot preloaded shot at right fender
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(shotTime));
        
        // backup and intake OurBall1
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path1))));
        runAction(new TurnToAngleAction(Units.radiansToDegrees(ball1ApproachHeadingRad)));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path2))));
        
        // turn and intake OurBall2
        runAction(new TurnToAngleAction(Units.radiansToDegrees(ball2ApproachHeadingRad)));
        runAction(new PathFollowerAction(path3));
        
        // return to fender and shoot
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new TurnToAngleAction(Units.radiansToDegrees(returnAngleRad)))));
        runAction(new PathFollowerAction(path4));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(shotTime));
        
        // head towards OurBall6
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path5))));
        runAction(new PathFollowerAction(path6));

    }
}
