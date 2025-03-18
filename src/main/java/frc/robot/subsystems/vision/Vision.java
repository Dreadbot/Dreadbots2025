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
	private double lastVisionTimestamp;
	private boolean inAutoAlign = false;


	public Vision(VisionConsumer consumer, PoseSupplier supplier, VisionIO io) {
		this.io = io;
		this.consumer = consumer;
		this.supplier = supplier;
		this.lastVisionPose = new Pose2d();
		this.lastVisionTimestamp = Double.POSITIVE_INFINITY;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Vision", inputs);
		ArrayList<Pose3d> tagPoses = new ArrayList<>();
		ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

		for(VisionObservation detection : inputs.detections) {
			
			Pose3d tagPose = VisionUtil.getApriltagPose(detection.id());
			double tagDist = tagPose.toPose2d().getTranslation().getDistance(detection.pose().getTranslation());
			boolean shouldRejectTag =
				detection.id() == 14 
				|| detection.id() == 15 
				|| detection.id() == 4 
				|| detection.id() == 5
				|| tagDist > 5.0
				|| detection.pose().getX() < 0.0
				|| detection.pose().getX() > VisionUtil.FIELD_LAYOUT.getFieldLength()
				|| detection.pose().getY() < 0.0
				|| detection.pose().getY() > VisionUtil.FIELD_LAYOUT.getFieldWidth()
				|| detection.pose().getTranslation().getDistance(supplier.getPose().getTranslation()) > 2.5;

			if(inAutoAlign){
				shouldRejectTag = shouldRejectTag 
				|| detection.id() == 1
				|| detection.id() == 2
				|| detection.id() == 3
				|| detection.id() == 12
				|| detection.id() == 13
				|| detection.id() == 16;
			}
			
			if(shouldRejectTag) {
				rejectedPoses.add(detection.pose());
				lastVisionPose = detection.pose();
				lastVisionTimestamp = detection.timestamp() / 1_000_000.0;
				continue;
			}
			double stdDevFactor = Math.pow(tagDist, 2.0);

			tagPoses.add(tagPose);


			double linearStdDev = VisionConstants.TRANSLATION_STD_DEV * stdDevFactor;
			double angularStdDev = VisionConstants.ROTATION_STD_DEV * stdDevFactor;

			double delay = (detection.timestamp() / 1_000_000.0) - (inputs.visionDelay + VisionConstants.TIMESTAMP_OFFSET);
			Logger.recordOutput("Vision/VisionPose", detection.pose());
			Logger.recordOutput("Vision/tagPoseLen", tagPoses.size());
			Logger.recordOutput("Vision/PoseTimestamp", delay);

			consumer.accept(detection.pose(), delay, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			lastVisionPose = detection.pose();
			lastVisionTimestamp = detection.timestamp() / 1_000_000.0;
		}
		Logger.recordOutput("Vision/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
		Logger.recordOutput("Vision/RejectedPoses", rejectedPoses.toArray(new Pose2d[rejectedPoses.size()]));

	}

	public void setInAutoAlign(boolean inAutoAlign) {
		this.inAutoAlign = inAutoAlign;
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
  public double getLastVisionTimestamp() {
	return lastVisionTimestamp;
  }
  @FunctionalInterface
  public static interface PoseSupplier {
	public Pose2d getPose();
  }
}
