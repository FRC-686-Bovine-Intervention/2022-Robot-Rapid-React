package frc.robot.Auto;

import frc.robot.Auto.AutoModes.AutoMode;
import frc.robot.lib.util.CrashTrackingRunnable;

public class AutoModeExecuter {
    private AutoMode autoMode;
    private Thread autoThread = null;
    
    public void setAutoMode(AutoMode _autoMode) 
    {
        autoMode = _autoMode;
    }

    public AutoMode getAutoMode() 
    {
        return autoMode;
    }

    public void start() 
    {

        if (autoThread == null) 
        {
            autoThread = new Thread(new CrashTrackingRunnable() 
            {
                @Override
                public void runCrashTracked() 
                {
                    if (autoMode != null) 
                    {
                        autoMode.run();
                    }
                }
            });
            autoThread.start();
        }

    }

    public void stop() 
    {
        if (autoMode != null) 
        {
            autoMode.stop();
        }
        autoThread = null;
    }
}
