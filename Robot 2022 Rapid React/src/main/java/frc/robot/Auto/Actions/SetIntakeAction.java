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
        Intake.getInstance().changeState(state);
    }
    
    @Override
    public void run() {
        Intake.getInstance().changeState(state);
    }

    @Override
    public boolean isFinished() {
        boolean finished = Intake.getInstance().calibrated && Intake.getInstance().isAtPos(state.armPos);
        if (finished) {
            //DEBUG
            System.out.println("Done with IntakeAction");         
        }
        return finished;        
    }

    @Override
    public void done() {
//DEBUG
System.out.println("Done with SetIntakeAction");        
    }
}
