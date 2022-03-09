package frc.robot.auto.modes;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Drive.VelocityHeadingSetpoint;
import frc.robot.subsystems.UnitNumber.Unit;

/**
 * Interface that holds all the field measurements 
 */

public class FieldDimensions 
{
	// dimensions of field components
	public static final double kFieldLengthX = 648;       // 54'
	public static final double kFieldLengthY = 324;       // 27'
    
    // Fender (left/right as viewed from the driver station)
    public static final double originToFenderCenterInches = 34.0;
    public static final Vector2d fenderCenter = new Vector2d(0,0);  // all autonomous coordinates are relative to center of fender
    public static final double fenderShotAngleRad = Units.degreesToRadians(180.0);  // shoot facing towards center of hub

    // Fender Shot Positions
    public static final Vector2d fenderShotPos = fenderCenter.add(new Vector2d(Constants.kCenterToFrontBumper, 0));
    public static final Vector2d fenderApproachPos = new Vector2d(48.0, 0);     // some distance in front of fender
    public static final Vector2d fenderBackupPos = new Vector2d(30.0, 0);       // some distance in front of fender

    //====================================================================================================
    // 2 Ball Auto 
    //====================================================================================================
    
    // Starting Pose
    public static final Vector2d lTarmacApex = new Vector2d(82.75, 0);   // outside of tape at (84.75, 0)
    public static final double lTarmacTapeAngle = Units.degreesToRadians(-112.5); 
    public static final double twoBallAutoInitialHeadingRad = Units.degreesToRadians(-22.5);
    public static final Pose twoBallAutoStartingPose = new Pose(new Vector2d(lTarmacApex
                                    .add(Vector2d.magnitudeAngle(Constants.kCenterToSideBumper, lTarmacTapeAngle))
                                    .sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, twoBallAutoInitialHeadingRad))), 
                                    twoBallAutoInitialHeadingRad);
    
    // Ball Positions
    public static final Vector2d ourBall4   = new Vector2d(116, -39);   // near left fender
    public static final Vector2d theirBall3 = new Vector2d(93, -85);    // left wall
    public static final Vector2d ourBall5   = new Vector2d(51, -127);   // other side of field, left wall
    
    
    //====================================================================================================
    // 3 Ball Auto 
    //====================================================================================================
    
    // Starting Pose
    public static final Pose threeBallAutoStartingPose = new Pose(fenderShotPos, Units.degreesToRadians(180));
    
    // Ball Positions
    public static final Vector2d ourBall1   = new Vector2d(116, 30);   // right wall
    public static final Vector2d ourBall2   = new Vector2d(93, -85);   // near center
    
}