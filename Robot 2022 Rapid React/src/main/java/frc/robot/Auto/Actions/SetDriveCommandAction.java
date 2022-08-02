package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.command_status.DriveCommand;
import frc.robot.subsystems.Drive;

public class SetDriveCommandAction implements Action{
    private DriveCommand cmd;
    private double timeToWait;
    private double startTime;

    public SetDriveCommandAction(DriveCommand cmd, double timeToWait)
    {
        this.cmd = cmd;
        this.timeToWait = timeToWait;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void run() {
        Drive.getInstance().setCommand(new DriveCommand(cmd.getLeftMotor(),cmd.getRightMotor()));
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }
    
    @Override
    public void done() {
        Drive.getInstance().setCommand(DriveCommand.COAST());
    }

    
}
