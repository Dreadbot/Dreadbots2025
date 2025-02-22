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
	private Pose2d lastVisionPose;

	public Vision(VisionConsumer consumer, VisionIO io) {
		this.io = io;
		this.consumer = consumer;
		this.lastVisionPose = new Pose2d();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Vision", inputs);
		for(VisionObservation detection : inputs.detections) {
			
			// std dev scaling goes here
			Logger.recordOutput("Vision/VisionPose", detection.pose());
			consumer.accept(detection.pose(), inputs.detections[0].timestamp() / 1_000_000.0, VisionConstants.STD_DEV);
			lastVisionPose = detection.pose();
		}
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
}
