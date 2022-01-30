package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Constants;

public class Controls {
    private static Controls instance;
    public static Controls getInstance() {if(instance == null){instance = new Controls();}return instance;}

    Joystick thrustmaster, buttonboard;

    public Controls()
    {
        thrustmaster =  new Joystick(Constants.kThrustmasterPort);
        buttonboard =   new Joystick(Constants.kButtonboardPort);
    }

    public enum JoystickEnum {THRUSTMASTER, BUTTONBOARD}
    
    public Vector2d getAxis(JoystickEnum joystick)
    {
        switch(joystick)
        {
            case THRUSTMASTER:  default:    return new Vector2d(-thrustmaster.getRawAxis(0),    -thrustmaster.getRawAxis(1));
            case BUTTONBOARD:               return new Vector2d(buttonboard.getRawAxis(0),      buttonboard.getRawAxis(1));
        }
    }

    public enum ButtonControlEnum {
        INTAKE,
        OUTTAKE,
        CLIMBERNEXTSTAGE,
        CLIMBERPREVSTAGE
    }
    
    public boolean getButton(ButtonControlEnum button)
    {
        switch(button)
        {
            case INTAKE:                    return thrustmaster.getRawButton(Thrustmaster.kTriggerButton);
            case OUTTAKE:                   return thrustmaster.getRawButton(Thrustmaster.kBottomThumbButton);
            case CLIMBERNEXTSTAGE:          return thrustmaster.getRawButton(Thrustmaster.kTopButton3);
            case CLIMBERPREVSTAGE:          return thrustmaster.getRawButton(Thrustmaster.kTopButton2);
            default:                        return false;
        }
    }
}
