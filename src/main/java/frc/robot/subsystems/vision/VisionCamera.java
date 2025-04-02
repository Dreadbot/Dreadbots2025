package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotState;
import frc.robot.RobotState.CurrentAction;
import frc.robot.subsystems.vision.VisionIO.VisionDetection;
import frc.robot.util.vision.VisionUtil;

public class VisionCamera {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final int index;
	private VisionConsumer consumer;
	private PoseSupplier supplier;
    private double linearStdDev;
	private double angularStdDev;
    private VisionDetection lastVisionDetection;

    public VisionCamera(VisionIO io, int index) {
		this.io = io;
        this.index = index;
		this.lastVisionDetection = new VisionDetection(new Pose2d(), 1, 0.0);
		this.linearStdDev =
			switch (index) {
				case 0 -> frontRightCameraLinearStdDevs;
				case 1 -> frontLeftCameraLinearStdDevs;
				case 2 -> backCameraLinearStdDevs;
				default -> 1_000_000;
			};
		this.angularStdDev =
			switch (index) {
				case 0 -> frontRightCameraAngularStdDevs;
				case 1 -> frontLeftCameraAngularStdDevs;
				case 2 -> backCameraAngularStdDevs;
				default -> 1_000_000;
			};
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("VisionCam" + Integer.toString(index), inputs);

        ArrayList<Pose3d> tagPoses = new ArrayList<>();
		ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

		for(VisionDetection detection : inputs.detections) {
			lastVisionDetection = detection;
			Pose3d tagPose = VisionUtil.getApriltagPose(detection.id());
			double tagDist = tagPose.toPose2d().getTranslation().getDistance(detection.pose().getTranslation());
			boolean shouldRejectTag =
				detection.id() == 14 
				|| detection.id() == 15 
				|| detection.id() == 4 
				|| detection.id() == 5
				|| detection.id() == 12
				|| detection.id() == 13
				|| detection.id() == 1
				|| detection.id() == 2
				|| tagDist > 5.0
				|| detection.pose().getX() < 0.0
				|| detection.pose().getX() > VisionUtil.FIELD_LAYOUT.getFieldLength()
				|| detection.pose().getY() < 0.0
				|| detection.pose().getY() > VisionUtil.FIELD_LAYOUT.getFieldWidth()
				|| detection.pose().getTranslation().getDistance(supplier.getPose().getTranslation()) > 1.5;

			if (RobotState.getInstance().getCurrentAction() == CurrentAction.AUTO_ALIGN || index == 2) { // If we are auto aligning, or this cam is the back cam, reject every non barge tags 
				shouldRejectTag = shouldRejectTag || VisionUtil.isNotReefId(detection.id());
			}
			
			if (shouldRejectTag) {
				rejectedPoses.add(detection.pose());
				continue;
			}
			double stdDevFactor = Math.pow(tagDist, 2.0);

			tagPoses.add(tagPose);

			// std dev scaling goes here
			Logger.recordOutput("Vision/Cam" + Integer.toString(index) + "/VisionPose" , detection.pose());
			Logger.recordOutput("Vision/Cam" + Integer.toString(index) + "/tagPoseLen", tagPoses.size());
			Logger.recordOutput("Vision/Cam" + Integer.toString(index) + "/PoseTimestamp", (detection.timestamp() / 1_000_000.0) - (inputs.visionDelay + VisionConstants.DELAY_OFFSET));

			consumer.accept(detection.pose(), (detection.timestamp() / 1_000_000.0) - (inputs.visionDelay + VisionConstants.DELAY_OFFSET), VecBuilder.fill(linearStdDev * stdDevFactor, linearStdDev * stdDevFactor, angularStdDev * stdDevFactor));
		}
		Logger.recordOutput("Vision/Cam" + Integer.toString(index) + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
		Logger.recordOutput("Vision/Cam" + Integer.toString(index)  + "/RejectedPoses", rejectedPoses.toArray(new Pose2d[rejectedPoses.size()]));
    }

	@FunctionalInterface
  	public static interface VisionConsumer {
		public void accept(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs);
	}
	public VisionDetection getLastVisionObservation() {
		return lastVisionDetection;
	}
	@FunctionalInterface
	public static interface PoseSupplier {
		public Pose2d getPose();
	}
	public void setConsumer(VisionConsumer consumer) {
		this.consumer = consumer;
	}
	public void setSupplier(PoseSupplier supplier) {
		this.supplier = supplier;
	}
}
