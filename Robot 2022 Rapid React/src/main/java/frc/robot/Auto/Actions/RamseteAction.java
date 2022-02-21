package frc.robot.auto.actions;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.RobotState;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.lib.util.Pose;
import frc.robot.subsystems.Drive;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class RamseteAction implements Action  
{
	Trajectory trajectory;
    Supplier<Pose2d> pose;
    RamseteController ramseteController;
    DifferentialDriveKinematics kinematics;
    BiConsumer<Double, Double> outputMetersPerSecond;

    Timer timer;

    public RamseteAction(Trajectory _trajectory) 
    {
    	trajectory = _trajectory;

        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.kTrackWidthInches));

        // Create and push Field2d to SmartDashboard.
        // m_field = new Field2d();
        // SmartDashboard.putData(m_field);

        // Push the trajectory to Field2d.
        // m_field.getObject("traj").setTrajectory(m_trajectory);           
    }

    @Override
    public void start() 
    {
		System.out.println("RamseteAction.start(), pose = " + RobotState.getInstance().getLatestFieldToVehicle().toString());

        Pose initialPose = new Pose(trajectory.getInitialPose().getX(), trajectory.getInitialPose().getY());
        RobotState.getInstance().reset(0,0,0,initialPose);

        timer = new Timer();
		timer.start();
    }


    @Override
    public void update() 
    {
    	if (timer.get() < trajectory.getTotalTimeSeconds())
        {
            // find current position
            Pose currentPose = RobotState.getInstance().getLatestFieldToVehicle();
            // convert to meters
            Pose2d currentPose2d = new Pose2d(Units.inchesToMeters(currentPose.getX()),
                                              Units.inchesToMeters(currentPose.getY()),
                                              new Rotation2d(currentPose.getHeading()));

            // get the position we should be at at this time
            Trajectory.State desiredPose = trajectory.sample(timer.get());

            // calculate wheel speeds needed to keep us on the trajectory
            ChassisSpeeds refChassisSpeeds = ramseteController.calculate(currentPose2d, desiredPose); 
            DifferentialDriveWheelSpeeds refWheelSpeeds = kinematics.toWheelSpeeds(refChassisSpeeds);
            // convert to inches per second
            WheelSpeed wheelSpeed = new WheelSpeed(Units.metersToInches(refWheelSpeeds.leftMetersPerSecond),
                                                   Units.metersToInches(refWheelSpeeds.rightMetersPerSecond));

            // set wheel speeds
            Drive.getInstance().setVelocitySetpoint(wheelSpeed);
        }
        else
        {
            Drive.getInstance().setOpenLoop(DriveCommand.COAST());
        }
	}	
	
	
    @Override
    public boolean isFinished() 
    {
    	return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void done() 
    {
		System.out.println("RamseteAction.done(),  pose = " + RobotState.getInstance().getLatestFieldToVehicle().toString());
		// cleanup code, if any
		timer.stop();
    }

 
    
    
    // public DataLogger getLogger() { return driveCtrl.getLogger(); }
}
