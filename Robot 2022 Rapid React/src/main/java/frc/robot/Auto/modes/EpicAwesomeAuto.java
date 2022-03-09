package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SetIntakeAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment.Options;
import frc.robot.lib.util.Vector2d;
import frc.robot.subsystems.Intake.IntakeState;

public class EpicAwesomeAuto extends AutoMode{
    public EpicAwesomeAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        Path path1 = new Path();
        Path path2 = new Path();
        Path path3 = new Path();
        Path rpath = new Path();
        Vector2d squareSize = new Vector2d(144,72);
        Vector2d topRight = new Vector2d(squareSize.getX(),0);
        Vector2d topLeft = new Vector2d(squareSize.getX(),squareSize.getY());
        Vector2d topLeftH = new Vector2d(squareSize.getX(),squareSize.getY()/2);
        Vector2d bottomLeft = new Vector2d(0,squareSize.getY());
        Vector2d bottomLeftH = new Vector2d(36,squareSize.getY());
        Vector2d bottomRight = new Vector2d(0,0);
        Options driveOptions = new Options(36,36,36,false);
        
        path1.add(new Waypoint(bottomRight, driveOptions));
        path1.add(new Waypoint(topRight, driveOptions));
        path1.add(new Waypoint(topLeftH, driveOptions));

        path2.add(new Waypoint(topLeftH, driveOptions));
        path2.add(new Waypoint(topLeft, driveOptions));
        path2.add(new Waypoint(bottomLeftH, driveOptions));
        
        path3.add(new Waypoint(bottomLeftH, driveOptions));
        path3.add(new Waypoint(bottomLeft, driveOptions));
        path3.add(new Waypoint(bottomRight, driveOptions));

        rpath.add(new Waypoint(bottomRight, driveOptions));
        rpath.add(new Waypoint(bottomLeft, driveOptions));
        rpath.add(new Waypoint(topLeft, driveOptions));
        rpath.add(new Waypoint(topRight, driveOptions));
        rpath.add(new Waypoint(bottomRight, driveOptions));
        rpath.setReverseDirection();
        
        runAction(new PathFollowerAction(path1));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.INTAKE),new PathFollowerAction(path2))));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE),new PathFollowerAction(path3))));
        runAction(new SetIntakeAction(IntakeState.OUTTAKE));
        runAction(new WaitAction(0.5));
        runAction(new ParallelAction(Arrays.asList(new SetIntakeAction(IntakeState.DEFENSE),new PathFollowerAction(rpath))));
    }
}
