package frc.robot.Auto.Actions;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Subsystems.Subsystems.Drive;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;

public class DriveStraight implements Action{

    private double startingDistance;
    private double distance;

    public DriveStraight(double distance)
    {
        this.distance = distance;
    }

    @Override
    public void start() {
        startingDistance = getDistance();
    }

    @Override
    public void run() {
        Drive.getInstance().setVelocityHeadingSetpoint(12, DriveState.getInstance().getHeadingDeg());
    }

    @Override
    public boolean isFinished() {
        return getDistance() - startingDistance >= distance;
    }

    @Override
    public void done() {
        Drive.getInstance().setCommand(DriveCommand.BRAKE());
    }

    private double getDistance()
    {
        return (DriveState.getInstance().getLeftDistanceInches() + DriveState.getInstance().getRightDistanceInches())/2;
    }
}
