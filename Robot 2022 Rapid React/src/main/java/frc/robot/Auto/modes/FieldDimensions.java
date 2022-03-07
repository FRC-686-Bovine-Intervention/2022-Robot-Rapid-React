package frc.robot.auto.modes;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Drive.VelocityHeadingSetpoint;

/**
 * Interface that holds all the field measurements 
 */

public class FieldDimensions 
{
	// dimensions of field components
	public static final double kFieldLengthX = 648;       // 54'
	public static final double kFieldLengthY = 324;       // 27'
    
    // Fender (left/right as viewed from the driver station)
    public static final double originToFenderCenterInches = 37.0;
    public static final double rFenderNormalAngleRad = Units.degreesToRadians(69);
    public static final double lFenderNormalAngleRad = rFenderNormalAngleRad - Units.degreesToRadians(90);
    public static final double rFenderApproachAngleRad = rFenderNormalAngleRad + Units.degreesToRadians(180);
    public static final double lFenderApproachAngleRad = lFenderNormalAngleRad + Units.degreesToRadians(180);
    public static final Vector2d rFenderCenter = Vector2d.magnitudeAngle(originToFenderCenterInches, rFenderNormalAngleRad);
    public static final Vector2d lFenderCenter = Vector2d.magnitudeAngle(originToFenderCenterInches, lFenderNormalAngleRad);

    // Fender Shot Positions
    public static final Pose rFenderShotPose = new Pose(rFenderCenter.add(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, rFenderNormalAngleRad)), rFenderApproachAngleRad);
    public static final Pose lFenderShotPose = new Pose(lFenderCenter.add(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, lFenderNormalAngleRad)), lFenderApproachAngleRad);

    // Ball Positions
    public static final double ballStartRingRadius = 153.0;
    public static final Vector2d ourBall1 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians( 80.25));   // near right fender and right wall 
    public static final Vector2d ourBall2 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians( 35.25));   // near right fender
    public static final Vector2d ourBall3 = new Vector2d(282.0, 117.75);                                                    // at terminal
    public static final Vector2d ourBall4 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians(-35.25));   // near left fender
    public static final Vector2d ourBall5 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians(-77.25));   // other side of field, left wall
    public static final Vector2d ourBall6 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians(125.25));   // other side of field, right wall
    
    public static final Vector2d theirBall1 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians(102.75)); // right wall
    public static final Vector2d theirBall2 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians( 12.75)); // center
    public static final Vector2d theirBall3 = Vector2d.magnitudeAngle(ballStartRingRadius, Units.degreesToRadians(-54.75)); // left wall

    // Tarmac apex
    public static final Vector2d lTarmacApex = new Vector2d(108.75, -41.75);   // from field CAD
    public static final double lTarmacTapeAngle = Units.degreesToRadians(46.8); // from field CAD
    public static final double ourBall4InitialHeadingRad = lTarmacTapeAngle - Math.PI/2;
    public static final Vector2d ourBall4FrontCenterOfRobot = lTarmacApex.sub(Vector2d.magnitudeAngle(Constants.kCenterToSideBumper, lTarmacTapeAngle));
    public static final Vector2d ourBall4CenterOfRotation = ourBall4FrontCenterOfRobot.sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, ourBall4InitialHeadingRad));

    // Starting Positions
    public static final Pose rFenderStartPose = rFenderShotPose;    // centered on fender
    public static final Pose lFenderStartPose = lFenderShotPose;    // centered on fender
    // the following 2 start poses assume the robot is at the tarmac tape, with the corner of the bumper at the apex of the tarmac
    public static final Pose ourBall1StartPose = new Pose(new Vector2d(25.9, 106.0).sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(90.0))), Units.degreesToRadians(90.0));
    // public static final Pose ourBall4StartPose = new Pose(ourBall4CenterOfRotation, ourBall4InitialHeadingRad);
    // measured from CAD
    public static final Pose ourBall4StartPose = new Pose(new Vector2d(86, -40), Units.degreesToRadians(-43.5));

    // Their Ball Dumping Positions
    public static final double dumpingDistanceInches = 12;
    // dumping to the right of the left fender wall, hidden from enemy view
    public static final Pose hubDumpingPose = new Pose(new Vector2d(39, 10).sub(Vector2d.magnitudeAngle(Constants.kCenterToIntake + dumpingDistanceInches, Units.degreesToRadians(135))), Units.degreesToRadians(135));
    // dumping against the wall in the hangar
    public static final Pose hangarDumpingPose = new Pose(new Vector2d(240, -160).sub(Vector2d.magnitudeAngle(Constants.kCenterToIntake + dumpingDistanceInches, Units.degreesToRadians(-90))), Units.degreesToRadians(-90));

    // Ending Positions
    // ourBall5EndPose assumes we are going from the left fender shot to ourBall5, avoiding their ball
    public static final Pose ourBall5EndPose = new Pose(new Vector2d(67.0, -103.0).sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(230.0))), Units.degreesToRadians(230.0));
    // ourBall6EndPose assumes we are going from the right fender shot to ourBall6, avoiding their ball
    public static final Pose ourBall6EndPose = new Pose(new Vector2d(-30.0, 110.0).sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, Units.degreesToRadians(165.0))), Units.degreesToRadians(165.0));
}