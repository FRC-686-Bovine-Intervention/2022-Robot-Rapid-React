package frc.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.ActionTestingAuto;
import frc.robot.auto.modes.AutoMode;
import frc.robot.auto.modes.EpicAwesomeAuto;
import frc.robot.auto.modes.OneBallAuto;
import frc.robot.command_status.RobotState;
import frc.robot.lib.util.Pose;

public class AutoManager {
    private static AutoManager instance;
    public static AutoManager getInstance() {if(instance == null){instance = new AutoManager();}return instance;}

    AutoModeExecuter autoModeExecuter = null;

    private ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    private SendableChooser<AutoMode> AutoModeChooser = new SendableChooser<>();
    private SendableChooser<Pose> InitialPoseChooser = new SendableChooser<>();
    private ComplexWidget wig = tab.add("mode", AutoModeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    private ComplexWidget a = tab.add("pose", InitialPoseChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    private AutoManager(){}

    public void InitChoices()
    {
        AutoModeChooser.addOption("OneBallAuto", new OneBallAuto());
        AutoModeChooser.addOption("BasicAuto", new BasicAuto());
        AutoModeChooser.setDefaultOption("TurnAround", new TurnAroundAuto());
        AutoModeChooser.setDefaultOption("WheelPosition", new WheelPositionAuto());
        InitialPoseChooser.setDefaultOption("new Pose()", new Pose());
    }

    public void init()
    {
        if (autoModeExecuter != null)
        {
            autoModeExecuter.stop();
        }
        autoModeExecuter = null;

        autoModeExecuter = new AutoModeExecuter();
        autoModeExecuter.setAutoMode(AutoModeChooser.getSelected());
        RobotState.getInstance().reset(InitialPoseChooser.getSelected());

        autoModeExecuter.start();
    }

    public void stop()
    {
        if (autoModeExecuter != null)
        {
            autoModeExecuter.stop();
        }
        autoModeExecuter = null;
    }
}
