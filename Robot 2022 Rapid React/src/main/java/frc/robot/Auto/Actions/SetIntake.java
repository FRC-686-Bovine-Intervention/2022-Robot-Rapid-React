package frc.robot.Auto.Actions;

import frc.robot.Subsystems.Subsystems.Intake;
import frc.robot.Subsystems.Subsystems.Intake.IntakeState;

public class SetIntake implements Action{
    private IntakeState set;

    public SetIntake(IntakeState set)
    {
        this.set = set;
    }
    
    @Override
    public void start() {
        
    }

    @Override
    public void run() {
        Intake.getInstance().changeState(set);
    }

    @Override
    public boolean isFinished() {
        return Intake.getInstance().isAtPos(set.armPos);
    }

    @Override
    public void done() {
        
    }
}
