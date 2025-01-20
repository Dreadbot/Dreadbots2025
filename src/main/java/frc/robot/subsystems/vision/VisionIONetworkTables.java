package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import frc.robot.util.vision.VisionPosition;

public class VisionIONetworkTables implements VisionIO {
    private StructArraySubscriber<VisionPosition> visionPositions;

    public VisionIONetworkTables() {
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        NetworkTable visionTable = ntinst.getTable(VisionConstants.FRONT_CAMERA_NAME);
        this.visionPositions = visionTable.getStructArrayTopic("visionPos", VisionPosition.struct).subscribe(new VisionPosition[]{}, PubSubOption.periodic(0.02));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        VisionObservation[] tmp = {};
        for (var i = 0; i < visionPositions.get().length; i++) {
            var currentPosition = visionPositions.get()[i];
            tmp[i] = new VisionObservation(new Pose2d(currentPosition.x, currentPosition.y, Rotation2d.kZero), visionPositions.getAtomic().timestamp, currentPosition.ID);
        }
        inputs.detections = tmp;
    }
}
