package frc.robot.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.RamseteAction;
import frc.robot.auto.actions.WaitAction;

/**
 * 2-Hatch Autonomous mode for Sandstorm period
 */

public class SimpleRamseteAuto extends AutoModeBase {

    Trajectory trajectory;

    public SimpleRamseteAuto() 
    { 
        Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        List<Translation2d> interiorWaypoints = List.of(new Translation2d(30, 30), new Translation2d(60, -30));
        Pose2d endPose = new Pose2d(90, 0, Rotation2d.fromDegrees(0));
        double maxVelocityInchesPerSecond = 90;
        double maxAccelerationInchesPerSecondsSquared = 90;
        TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(maxVelocityInchesPerSecond), Units.inchesToMeters(maxAccelerationInchesPerSecondsSquared));
        trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);

     
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        runAction(new WaitAction(0.5));
        runAction(new RamseteAction(trajectory));
    }

}
