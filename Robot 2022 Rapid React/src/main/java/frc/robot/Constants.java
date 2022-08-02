package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public static int kLeftMasterID =       1;
    public static int kLeftSlaveID =        2;
    public static int kRightMasterID =      3;
    public static int kRightSlaveID =       4;
    // Control Hardware
    public static int kThrustmasterPort =   0;
    public static int kButtonboardPort =    1;

    public static double kLoopDt = 0.01;
    public static int kTalonTimeoutMs = 5;


    // Robot Dimensions
    public static double kCenterToSideBumper = 15.0;
    public static double kCenterToFrontBumper = 19.5;
    public static double kCenterToIntake = 32.0;
}

