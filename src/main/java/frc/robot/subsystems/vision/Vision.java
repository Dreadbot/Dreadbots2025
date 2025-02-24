package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionObservation;
import frc.robot.util.vision.VisionUtil;

public class Vision extends SubsystemBase {
	private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
	private final VisionIO io;
	private final VisionConsumer consumer;
	private final PoseSupplier supplier;
	private Pose2d lastVisionPose;

	public Vision(VisionConsumer consumer, PoseSupplier supplier, VisionIO io) {
		this.io = io;
		this.consumer = consumer;
		this.supplier = supplier;
		this.lastVisionPose = new Pose2d();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Vision", inputs);
		ArrayList<Pose3d> tagPoses = new ArrayList<>();
		for(VisionObservation detection : inputs.detections) {
			Pose3d tagPose = VisionUtil.getApriltagPose(detection.id());
			double tagDist = tagPose.toPose2d().getTranslation().getDistance(supplier.getPose().getTranslation());
			double stdDevFactor = Math.pow(tagDist, 1.5);

			tagPoses.add(tagPose);

			double linearStdDev = VisionConstants.TRANSLATION_STD_DEV * stdDevFactor;
			double angularStdDev = VisionConstants.ROTATION_STD_DEV * stdDevFactor;

			// std dev scaling goes here
			Logger.recordOutput("Vision/VisionPose", detection.pose());
			Logger.recordOutput("Vision/PoseTimestamp", detection.timestamp() / 1_000_000.0);

			consumer.accept(detection.pose(), inputs.detections[0].timestamp() / 1_000_000.0, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			lastVisionPose = detection.pose();
		}
		Logger.recordOutput("Vision/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
	}


@FunctionalInterface
public static interface VisionConsumer {
	public void accept(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs);
  }
  public Pose2d getLastVisionPose() {
	return lastVisionPose;
  }
  @FunctionalInterface
  public static interface PoseSupplier {
	public Pose2d getPose();
  }
}
