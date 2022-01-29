package frc.robot.Subsystems.Subsystems;

import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Climber subsystem</h4>
 * Has 4 states: <b>Defence</b>, <b>Extend</b>, <b>Retract</b>, and <b>Lock</b><p>
 * <b>Defence</b> is when the robot is not currently climbing, and the climber is in the resting state<p>
 * <b>Extend</b> is when the climber extends and gives driver control of the pivot arms after extending when the previous state is not Defence<p>
 * <b>Retract</b> is when the climber retracts and the pivot arms move back<p>
 * <b>Lock</b> is when the pivot arms move forward to engage the bar, the climber extends a little to transfer support to the pivot arms, and the pivot arms move forward to rotate the robot<p>
*/
public class Climber extends Subsystem {
    private static Climber instance = null;
    //@Override
    public Climber getInstance() {if(instance == null){instance = new Climber();}return instance;}

    @Override
    public void init()
    {

    }
    @Override
    public void run()
    {

    }
}
