package frc.robot.auto.actions;

import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.subsystems.Drive;

public class TurnToAngle implements Action{

    private double targetAngleDeg;

    private static final double angleErrorThresholdDeg = 1.0;
    private static final double wheelVelocityThresholdInchesPerSec = 5;

    public TurnToAngle(double _targetAngleDeg)
    {
        targetAngleDeg = _targetAngleDeg;
    }

    @Override
    public void start() {
    }

    @Override
    public void run() {
        Drive.getInstance().setTurnToHeadingSetpoint(targetAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(targetAngleDeg - DriveState.getInstance().getHeadingDeg()) < angleErrorThresholdDeg) &&
               (Math.abs(DriveState.getInstance().getLeftSpeedInchesPerSec()) < wheelVelocityThresholdInchesPerSec) &&
               (Math.abs(DriveState.getInstance().getRightSpeedInchesPerSec()) < wheelVelocityThresholdInchesPerSec);
    }

    @Override
    public void done() {
        Drive.getInstance().setOpenLoop(DriveCommand.BRAKE());  // brake so that we stop near target angle
    }
}
