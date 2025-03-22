package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionCamera.PoseSupplier;
import frc.robot.subsystems.vision.VisionCamera.VisionConsumer;
import frc.robot.subsystems.vision.VisionIO.VisionDetection;

public class Vision extends SubsystemBase {
	private VisionDetection lastVisionObservation;
	private final List<VisionCamera> cameras;

	public Vision(List<VisionCamera> cameras, VisionConsumer consumer, PoseSupplier supplier) {
		this.cameras = cameras;
		this.lastVisionObservation = new VisionDetection(new Pose2d(), 0, 0.0);;
		for (VisionCamera camera : cameras) {
			camera.setConsumer(consumer);
			camera.setSupplier(supplier);
		}
	}

	@Override
	public void periodic() {
		ArrayList<VisionDetection> lastVisionObservations = new ArrayList<VisionDetection>();
		for (VisionCamera camera : cameras) {
			camera.periodic();
			lastVisionObservations.add(camera.getLastVisionObservation());
		}
		for (VisionDetection observation : lastVisionObservations) {
			if (observation.timestamp() > lastVisionObservation.timestamp()) {
				lastVisionObservation = observation;
			}
		}
	}

	public Pose2d getLastVisionPose() {
		return lastVisionObservation.pose();
	}
}
