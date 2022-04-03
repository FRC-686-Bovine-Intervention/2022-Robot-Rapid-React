package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public static final int kPigeonID =           1;
    public static final int kLeftMasterID =       2;
    public static final int kLeftSlaveID =        3;
    public static final int kRightMasterID =      4;
    public static final int kRightSlaveID =       5;
    // Intake Hardware
    public static final int kArmMotorID =         6;
    public static final int kRollerMotorID =      7;
    // Climber Hardware
    public static final int kLeftClimberID =      8;
    public static final int kRightClimberID =     9;
    // Control Hardware
    public static final int kThrustmasterPort =   0;
    public static final int kButtonboardPort =    1;

    public static final double kLoopDt = 0.01;
    public static final int kTalonTimeoutMs = 5;

    public static final double kCameraFrameRate = 90;

    // Robot Dimensions
    public static final double kCenterToSideBumper = 15.0;
    public static final double kCenterToFrontBumper = 19.5;
    public static final double kCenterToIntake = 32.0;

    public static final double kCameraPoseX = -7;
    public static final double kCameraPoseY = 0;
    public static final double kCameraPoseZ = 36;
    public static final double kCameraPosePitchRad = Math.toRadians(-15);


    public static final double kBallDiameter = 9.25;
}

