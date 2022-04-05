package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class Vision {
    private static Vision instance; 
    public static Vision getInstance() {if(instance == null){instance = new Vision();}return instance;} 
    private Vision(){}

    public void init()
    {
      // UsbCamera limelight = CameraServer.startAutomaticCapture();
      UsbCamera driverCamera = CameraServer.startAutomaticCapture();
      // limelight.setResolution(320, 240);
      driverCamera.setResolution(320, 240);
    }
}
