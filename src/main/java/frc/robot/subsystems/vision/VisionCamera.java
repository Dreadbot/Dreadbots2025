package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.VisionObservation;
import frc.robot.util.vision.VisionUtil;

public class VisionCamera {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final int index;
	private final VisionConsumer consumer;
	private final PoseSupplier supplier;
    private double linearStdDev;
	private double angularStdDev;
    private Pose2d lastVisionPose;

    public VisionCamera(VisionConsumer consumer, PoseSupplier supplier, VisionIO io, int index, double linearStdDev, double angularStdDev) {
        this.consumer = consumer;
		this.supplier = supplier;
		this.io = io;
        this.index = index;
        this.linearStdDev = linearStdDev;
		this.angularStdDev = angularStdDev;
		this.lastVisionPose = new Pose2d();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("VisionCam" + Integer.toString(index), inputs);

        ArrayList<Pose3d> tagPoses = new ArrayList<>();
		ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

		for(VisionObservation detection : inputs.detections) {
			lastVisionPose = detection.pose();
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
				|| detection.pose().getTranslation().getDistance(supplier.getPose().getTranslation()) > 1.5;
			if(shouldRejectTag) {
				rejectedPoses.add(detection.pose());
				continue;
			}
			double stdDevFactor = Math.pow(tagDist, 2.0);

			tagPoses.add(tagPose);

			// std dev scaling goes here
			Logger.recordOutput("Vision/VisionPose" + Integer.toString(index), detection.pose());
			Logger.recordOutput("Vision/tagPoseLen" + Integer.toString(index), tagPoses.size());
			Logger.recordOutput("Vision/PoseTimestamp" + Integer.toString(index), (detection.timestamp() / 1_000_000.0) - inputs.visionDelay);

			consumer.accept(detection.pose(), (detection.timestamp() / 1_000_000.0) - inputs.visionDelay, VecBuilder.fill(linearStdDev * stdDevFactor, linearStdDev * stdDevFactor, angularStdDev * stdDevFactor));
		}
		Logger.recordOutput("Vision/TagPoses" + Integer.toString(index), tagPoses.toArray(new Pose3d[tagPoses.size()]));
		Logger.recordOutput("Vision/RejectedPoses" + Integer.toString(index), rejectedPoses.toArray(new Pose2d[rejectedPoses.size()]));
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
