package frc.robot.vision;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.loops.Loop;

/**
 * VisionLoop contains the various attributes calculated by the vision system,
 * namely a list of targets and the timestamp at which it was captured.
 */
public class VisionLoop implements Loop {
	private static VisionLoop instance = new VisionLoop();

	public static VisionLoop getInstance() {
		return instance;
	}

	private VisionLoop()
	{
		ballCamera = new PhotonCamera("BallCamera");//mmal_service_16.1
	}

	// camera selection
	public PhotonCamera ballCamera;

	public VisionTargetList visionTargetList = VisionTargetList.getInstance();

	@Override
	public void onStart() {
		// nothing
	}

	@Override
	public void onLoop() {
		System.out.println(ballCamera.getPipelineIndex());
		double currentTime = Timer.getFPGATimestamp();

		// get target info from Limelight
		getTargets(currentTime);
		ballCamera.setPipelineIndex(DriverStation.getAlliance() == Alliance.Red ? 0 : 1);
	}

	@Override
	public void onStop() {
		// nothing
	}

	public void getTargets(double currentTime) {
		double cameraLatency = ballCamera.getLatestResult().getLatencyMillis() / 1000.0;
		double imageCaptureTimestamp = currentTime - cameraLatency; // assumes transport time from phone to this code is
																	// instantaneous

		ArrayList<VisionTargetList.Target> targets = new ArrayList<>();	// initially empty

		PhotonPipelineResult latest = ballCamera.getLatestResult();
		if (latest.hasTargets()) 
		{
			double hAngle = latest.getBestTarget().getYaw();
			double vAngle = latest.getBestTarget().getPitch();
			VisionTargetList.Target target = new VisionTargetList.Target(hAngle, vAngle);
			targets.add(target);
		}

		visionTargetList.set(imageCaptureTimestamp, targets);
	}


	/**
	 * @param visionTargetList the visionTargetList to set
	 */
	public void setVisionTargetList(VisionTargetList visionTargetList) {
		this.visionTargetList = visionTargetList;
	}

}
