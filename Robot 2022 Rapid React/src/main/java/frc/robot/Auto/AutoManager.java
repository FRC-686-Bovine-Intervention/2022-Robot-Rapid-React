package frc.robot.Auto;

public class AutoManager {
    private static AutoManager instance;
    public static AutoManager getInstance() {if(instance == null){instance = new AutoManager();}return instance;}

    AutoModeExecuter autoModeExecuter = null;

    private AutoManager(){}

    public void InitChoices()
    {

    }

    public void init()
    {
        if (autoModeExecuter != null)
        {
            autoModeExecuter.stop();
        }
        autoModeExecuter = null;

        autoModeExecuter.setAutoMode();
        setInitialPose();

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
