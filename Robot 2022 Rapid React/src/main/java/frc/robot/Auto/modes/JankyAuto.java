package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetDriveCommandAction;
import frc.robot.command_status.DriveCommand;

public class JankyAuto extends AutoMode{

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetDriveCommandAction(new DriveCommand(-0.3, -0.3), 2));
        runAction(new SetDriveCommandAction(new DriveCommand(0.5, -0.5), 13));
    }
    
}
