package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionCamera.PoseSupplier;
import frc.robot.subsystems.vision.VisionCamera.VisionConsumer;
import frc.robot.subsystems.vision.VisionIO.VisionDetection;

public class Vision extends SubsystemBase {
	private VisionDetection lastVisionDetection;
	private final List<VisionCamera> cameras;

	public Vision(List<VisionCamera> cameras, VisionConsumer consumer, PoseSupplier supplier) {
		this.cameras = cameras;
		this.lastVisionDetection = new VisionDetection(new Pose2d(), 0, 0.0);;
		for (VisionCamera camera : cameras) {
			camera.setConsumer(consumer);
			camera.setSupplier(supplier);
		}
	}

	@Override
	public void periodic() {
		ArrayList<VisionDetection> lastVisionDetections = new ArrayList<VisionDetection>();
		for (VisionCamera camera : cameras) {
			camera.periodic();
			lastVisionDetections.add(camera.getLastVisionObservation());
		}
		for (VisionDetection observation : lastVisionDetections) {
			if (observation.timestamp() > lastVisionDetection.timestamp()) {
				lastVisionDetection = observation;
			}
		}
	}

	public Pose2d getLastVisionPose() {
		return lastVisionDetection.pose();
	}

	public VisionDetection getLastVisionDetection() {
		return lastVisionDetection;
	}
}
