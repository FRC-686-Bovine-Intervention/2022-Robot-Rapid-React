package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public static int kLeftMasterID =       1;
    public static int kLeftSlaveID =        1;
    public static int kRightMasterID =      1;
    public static int kRightSlaveID =       1;
    // Intake Hardware

    // Climber Hardware

    // Control Hardware
    public static int kThrustmasterPort =   0;
    public static int kButtonboardPort =    1;
}
