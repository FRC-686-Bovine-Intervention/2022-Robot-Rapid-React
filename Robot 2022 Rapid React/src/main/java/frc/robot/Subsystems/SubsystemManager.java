package frc.robot.Subsystems;

import java.util.ArrayList;

import frc.robot.Subsystems.Subsystems.*;

public class SubsystemManager {
    private static SubsystemManager instance;
    public static SubsystemManager getInstance() {if(instance == null){instance = new SubsystemManager();}return instance;}

    public ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void init()
    {
        subsystems.add(Drivetrain.getInstance());
        //subsystems.add(Climber.getInstance());
        subsystems.add(Intake.getInstance());
    }

    public void run()                   {for (Subsystem s : subsystems) {if (s.Enabled){s.run();}}}
    public void runTestMode()           {for (Subsystem s : subsystems) {if (s.Enabled){s.runTestMode();}}}
    public void runCalibration()        {for (Subsystem s : subsystems) {if (s.Enabled){s.runCalibration();}}}
    public void updateShuffleboard()    {for (Subsystem s : subsystems) {s.updateShuffleboard();}}
}
