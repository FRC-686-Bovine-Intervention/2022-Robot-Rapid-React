package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.loops.DriveLoop;

/**
 * 2-Hatch Autonomous mode for Sandstorm period
 */

public class SimplePathFollowerAuto extends AutoModeBase {

    public SimplePathFollowerAuto() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        double speed = 48;   //DriveLoop.kPathFollowingMaxVel;
        double accel = 24;   //DriveLoop.kPathFollowingMaxAccel
        double lookaheadDist = DriveLoop.kPathFollowingLookahead;

        PathSegment.Options pathOptions =  new PathSegment.Options(speed, accel, lookaheadDist, false);

        Pose startPose = new Pose(0.0, 0.0, 0.0);
        Vector2d startPosition = startPose.getPosition();
        Vector2d endPosition = new Vector2d(60.0, 0.0);

    
        Path path = new Path();    // no backup
        path.add(new Waypoint(startPosition, pathOptions));       // drive slowly off of hab
        path.add(new Waypoint(endPosition,   pathOptions));

 
        runAction(new WaitAction(0.5));               // initial delay (optional)
        runAction(new PathFollowerAction(path));

        // Done!

    }


}
