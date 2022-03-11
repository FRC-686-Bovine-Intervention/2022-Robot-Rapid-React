package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class SetIntakeAction implements Action{
    private IntakeState state;

    public SetIntakeAction(IntakeState state)
    {
        this.state = state;
    }
    
    @Override
    public void start() {
        Intake.getInstance().setState(state);
    }
    
    @Override
    public void run() {
        Intake.getInstance().setState(state);
    }

    @Override
    public boolean isFinished() {
        boolean finished = Intake.getInstance().calibrated && Intake.getInstance().isAtPos(state.armPos);
        return finished;        
    }

    @Override
    public void done() {
        System.out.println("SetIntake.done()");
    }
}
