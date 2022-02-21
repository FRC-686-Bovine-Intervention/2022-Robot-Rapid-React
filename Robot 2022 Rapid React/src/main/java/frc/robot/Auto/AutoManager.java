package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoManager {
    private static AutoManager instance;
    public static AutoManager getInstance() {if(instance == null){instance = new AutoManager();}return instance;}

    private AutoManager(){}

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public void InitChoices()
    {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto/Auto choices", m_chooser);
    }

    public void init()
    {
        m_autoSelected = m_chooser.getSelected();
    }

    public void run()
    {
        
    }
}
