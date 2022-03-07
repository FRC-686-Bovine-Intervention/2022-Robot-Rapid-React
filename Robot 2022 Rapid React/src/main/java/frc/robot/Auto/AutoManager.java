package frc.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.*;

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
        AutoModeChooser.addOption("Action Testing", new ActionTestingAuto());
        AutoModeChooser.addOption("TurnAround", new TurnAroundAuto());
        AutoModeChooser.addOption("WheelPosition", new WheelPositionAuto());
        AutoModeChooser.addOption("3 Ball Auto", new ThreeBallAuto());
        AutoModeChooser.setDefaultOption("2 Ball Auto", new TwoBallAuto());
        InitialPoseChooser.addOption("new Pose()", new Pose());
        InitialPoseChooser.setDefaultOption("Right Fender", FieldDimensions.rFenderStartPose);
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
