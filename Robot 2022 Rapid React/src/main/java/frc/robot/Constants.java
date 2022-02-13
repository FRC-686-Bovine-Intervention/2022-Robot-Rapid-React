package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public static int kLeftMasterID =       2;
    public static int kLeftSlaveID =        3;
    public static int kRightMasterID =      4;
    public static int kRightSlaveID =       5;
    // Intake Hardware
    public static int kArmMotorID =         6;
    public static int kRollerMotorID =      7;
    // Climber Hardware
    public static int kLeftClimberID =      0;
    public static int kRightClimberID =     0;
    // Control Hardware
    public static int kThrustmasterPort =   0;
    public static int kButtonboardPort =    1;

    public static final int kCANTimeoutMs = 10;         // ms, use for on the fly updates
    public static final int kLongCANTimeoutMs = 100;    // ms, use for constructors
}
