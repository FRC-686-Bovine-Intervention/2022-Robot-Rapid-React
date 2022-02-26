package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveStraight;
import frc.robot.auto.actions.SetIntake;
import frc.robot.subsystems.Intake;

public class BasicAuto extends AutoMode{
    public BasicAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("BasicAuto.routine()");
        runAction(new DriveStraight(8*12));

        //runAction(new SetIntake(Intake.IntakeState.INTAKE));
        //runAction(new SetIntake(Intake.IntakeState.OUTTAKE));
    }
}
