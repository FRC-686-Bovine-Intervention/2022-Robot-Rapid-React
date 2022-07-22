package frc.robot.auto.modes;

import java.util.Arrays;

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
import frc.robot.lib.util.Pose;
import frc.robot.subsystems.Intake.IntakeState;

public class TrollAuto extends AutoMode{
    public TrollAuto(){
        initialPose = new Pose();
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        double maxSpeed = 36;
        double accel = 36;
        double lookaheadDist = 24;
        boolean visionEnabled = false;
        Options driveOptions = new Options(maxSpeed, accel, lookaheadDist, visionEnabled);
        
        double shotTime = 0.5;

        Path path = new Path();
        path.add(new Waypoint(initialPose, driveOptions));
        path.add(new Waypoint(new Pose(0,-48), driveOptions));
        path.setReverseDirection();

        //================================================================
        // TROLL AUTO
        //================================================================
        RobotState.getInstance().reset(initialPose);
        runAction(new WaitAction(AutoManager.autoInitialDelaySec)); 

        runAction(new SetIntakeAction(IntakeState.OUTTAKE_GROUND));
        runAction(new WaitAction(shotTime));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE), new TurnToAngleAction(-90))));
        runAction(new PathFollowerAction(path));
    }
}
