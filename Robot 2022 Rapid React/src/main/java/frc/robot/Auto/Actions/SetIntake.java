package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class SetIntake implements Action{
    private IntakeState set;

    public SetIntake(IntakeState set) //
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
        return Intake.getInstance().calibrated && Intake.getInstance().isAtPos(set.armPos);
    }

    @Override
    public void done() {
        
    }
}
