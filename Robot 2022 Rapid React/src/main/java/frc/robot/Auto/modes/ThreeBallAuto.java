package frc.robot.auto.modes;

import java.util.Arrays;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

public class ThreeBallAuto extends AutoMode{
    public ThreeBallAuto(){
        initialPose = FieldDimensions.threeBallAutoStartingPose;
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        double maxSpeed = 24;//36;
        double accel = 24;//36;
        double lookaheadDist = 24;
        boolean visionEnabled = false;
        Options driveOptions = new Options(maxSpeed, accel, lookaheadDist, visionEnabled);

        double shotTime = 0.3;

        // path1: after initial shot, backup so we can turn around
        Vector2d initialPos = initialPose.getPosition();
        Path path1 = new Path();
        path1.add(new Waypoint(initialPos, driveOptions));
        path1.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path1.setReverseDirection();
        
        // path2: drive to ball 1
        double ball2of3ApproachHeadingRad = FieldDimensions.ball2of3[DriverStation.getAlliance().ordinal()].sub(FieldDimensions.fenderBackupPos).angle();
        Vector2d ourBall2of3IntakePos = FieldDimensions.ball2of3[DriverStation.getAlliance().ordinal()].sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper+6, ball2of3ApproachHeadingRad)); 
        Path path2 = new Path();
        path2.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path2.add(new Waypoint(ourBall2of3IntakePos, driveOptions));

        // path3: drive to ball 2
        double ball3of3ApproachHeadingRad = FieldDimensions.ball3of3[DriverStation.getAlliance().ordinal()].sub(ourBall2of3IntakePos).angle();                   //chocolate fudge
        Vector2d ourBall3of3IntakePos = FieldDimensions.ball3of3[DriverStation.getAlliance().ordinal()].sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper - 12, ball3of3ApproachHeadingRad)); 
        Path path3 = new Path();
        path3.add(new Waypoint(ourBall2of3IntakePos, driveOptions));
        path3.add(new Waypoint(ourBall3of3IntakePos, driveOptions));

        // path4: go back to the fender and shoot
        double returnAngleRad = FieldDimensions.fenderApproachPos.sub(ourBall3of3IntakePos).angle();
        Path path4 = new Path();
        path4.add(new Waypoint(ourBall3of3IntakePos, driveOptions));
        path4.add(new Waypoint(FieldDimensions.fenderApproachPos, driveOptions));
        path4.add(new Waypoint(FieldDimensions.fenderShotPos, driveOptions));

        // path5 backup
        Path path5 = new Path();
        path5.add(new Waypoint(FieldDimensions.fenderShotPos, driveOptions));
        path5.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path5.setReverseDirection();
        
        // path6: head towards final ball
        double finalHeadingRad = FieldDimensions.threeBallAutoFinalTarget.sub(FieldDimensions.fenderBackupPos).angle();
        double finalTravelDist = 24;
        Vector2d finalPos = FieldDimensions.fenderBackupPos.add(Vector2d.magnitudeAngle(finalTravelDist, finalHeadingRad));
        Path path6 = new Path();
        path6.add(new Waypoint(FieldDimensions.fenderBackupPos, driveOptions));
        path6.add(new Waypoint(finalPos, driveOptions));


        //================================================================
        // THREE BALL AUTO
        //================================================================
        
        // set initial pose
        RobotState.getInstance().reset(initialPose);

        runAction(new WaitAction(AutoManager.autoInitialDelaySec)); 

        // shoot preloaded shot
        runAction(new SetIntakeAction(IntakeState.DEFENSE));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(shotTime));
        
        // backup and intake ball 2 of 3
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path1))));
        runAction(new TurnToAngleAction(Units.radiansToDegrees(ball2of3ApproachHeadingRad)));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE), new PathFollowerAction(path2))));
        
        // turn and intake OurBall2
        runAction(new TurnToAngleAction(Units.radiansToDegrees(ball3of3ApproachHeadingRad)));
        runAction(new PathFollowerAction(path3));
        
        // return to fender and shoot
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new TurnToAngleAction(Units.radiansToDegrees(returnAngleRad)))));
        runAction(new PathFollowerAction(path4));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(shotTime));
        
        // head towards OurBall6
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new PathFollowerAction(path5))));
        runAction(new TurnToAngleAction(Units.radiansToDegrees(finalHeadingRad)));
        runAction(new PathFollowerAction(path6));

    }
}
