package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

public class VisionTargetList
{
	/*
	 * The VisionTargetListState keeps track of multiple targets
	 */

	private static VisionTargetList instance = new VisionTargetList();
	public static VisionTargetList getInstance() { return instance; }	

	private List<Target> targets = new ArrayList<>();
	private double imageCaptureTimestamp = 0;
	
	
	// Synchronized get/set functions for access from other threads
	
	synchronized public void set(double _imageCaptureTimestamp, List<Target> _targets)
	{ 
		imageCaptureTimestamp = _imageCaptureTimestamp;
		targets = _targets; 
	}
	synchronized public List<Target> getTargets() 			{ return targets; }
	synchronized public double getImageCaptureTimestamp()	{ return imageCaptureTimestamp; }



	public static class Target
	{
	/*
	 * A container class for Targets detected by the vision system, containing the
	 * horizontal and vertical angles from the optical axis.
	 */
		protected double hCenter; 	// horizontal angle to center of target from optical axis, in radians
		protected double vCenter; 	//   vertical angle to center of target from optical axis, in radians

		public Target(double _hCenter, double _vCenter)
		{
			hCenter = _hCenter;
			vCenter = _vCenter;
		}

		public double getHorizontalAngle()	{ return hCenter; }
		public double getVerticalAngle()	{ return vCenter; }

		public String toString()
		{
			return "hCenter:" + hCenter + ", vCenter:" + vCenter + ".";
		}
	}
}