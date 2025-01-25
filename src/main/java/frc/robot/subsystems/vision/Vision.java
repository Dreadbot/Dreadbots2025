package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
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

	public Vision(VisionConsumer consumer, VisionIO io) {
		this.io = io;
		this.consumer = consumer;
		NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
		NetworkTable visionTable = ntinst.getTable(VisionConstants.FRONT_CAMERA_NAME);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Vision", inputs);
		for(VisionObservation detection : inputs.detections) {
			Pose2d detectionInWorldAxis = VisionUtil.tagAxisToWorldAxis(
				new Pose3d(detection.pose()), 
				VisionUtil.getApriltagPose(detection.tagId())
			);
			Pose2d poseEstimate = VisionUtil.calculatePoseFromTagOffset(detectionInWorldAxis, detection.tagId());

			// std dev scaling goes here

			consumer.accept(poseEstimate, inputs.detections[0].timestamp(), VisionConstants.STD_DEV);
		}
	}


@FunctionalInterface
public static interface VisionConsumer {
	public void accept(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
