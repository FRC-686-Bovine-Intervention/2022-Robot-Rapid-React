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

    public void run()
    {
        for (Subsystem s : subsystems) {s.run();}
    }

    public void runCalibration()
    {
        for (Subsystem s : subsystems) {s.runCalibration();}
    }

    public void updateSmartDashboard() {for (Subsystem s : subsystems) {s.updateSmartDashboard();}}
}
