package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionObservation;
import frc.robot.util.vision.VisionUtil;

public class Vision extends SubsystemBase {
	private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
	private final VisionIO io;
	private final VisionConsumer consumer;
	private final RotationSupplier robotRotation;


	public Vision(VisionConsumer consumer, RotationSupplier robotRotation, VisionIO io) {
		this.io = io;
		this.consumer = consumer;
		this.robotRotation = robotRotation;
		SmartDashboard.putNumber("TagX", 0.0);
		SmartDashboard.putNumber("TagY", 0.0);
		SmartDashboard.putNumber("TagId", -1.0);

	}


	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Vision", inputs);
		if(SmartDashboard.getNumber("TagId", -1) != -1) {
			var pose = new Pose2d(SmartDashboard.getNumber("TagX", 0 ), SmartDashboard.getNumber("TagY", 0), Rotation2d.kZero);
			var detection = new VisionObservation(pose, Timer.getFPGATimestamp(), (int) SmartDashboard.getNumber("TagId", 0));
			inputs.detections = new VisionObservation[1];
			inputs.detections[0] = detection;
		} // debugging code

		for(VisionObservation detection : inputs.detections) {
			Pose2d detectionInWorldAxis = VisionUtil.tagAxisToWorldAxis(
				new Pose3d(detection.pose()),
				robotRotation.getRotation()
			);
			Pose2d poseEstimate = VisionUtil.calculatePoseFromTagOffset(detectionInWorldAxis, detection.tagId());
			Logger.recordOutput("Vision/pose", poseEstimate);
			Logger.recordOutput("Vision/tagOffsetWorld", detectionInWorldAxis);
			Logger.recordOutput("Vision/tagOffset", detection.pose());


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
	@FunctionalInterface
	public static interface RotationSupplier {
		public Rotation2d getRotation();
	} 
}
