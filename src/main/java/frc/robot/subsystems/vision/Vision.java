package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
	private Pose2d lastVisionPose;
	private final List<VisionCamera> cameras;

	public Vision(List<VisionCamera> cameras) {
		this.cameras = cameras;
		this.lastVisionPose = new Pose2d();
	}

	@Override
	public void periodic() {
		ArrayList<Pose2d> lastVisionPoses = new ArrayList<Pose2d>();
		for (VisionCamera camera : cameras) {
			camera.periodic();
			lastVisionPoses.add(camera.getLastVisionPose());
		}
	}

	public Pose2d getLastVisionPose() {
		return lastVisionPose;
	}
}
