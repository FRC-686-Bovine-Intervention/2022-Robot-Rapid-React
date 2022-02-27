package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class SetIntake implements Action{
    private IntakeState state;

    public SetIntake(IntakeState state)
    {
        this.state = state;
    }
    
    @Override
    public void start() {
        Intake.getInstance().changeState(state);
    }
    
    @Override
    public void run() {
        Intake.getInstance().changeState(state);
    }

    @Override
    public boolean isFinished() {
        return Intake.getInstance().calibrated && Intake.getInstance().isAtPos(state.armPos);
    }

    @Override
    public void done() {
        
    }
}
