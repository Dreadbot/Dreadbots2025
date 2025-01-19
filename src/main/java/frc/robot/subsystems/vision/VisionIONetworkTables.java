package frc.robot.subsystems.vision;

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
        VisionPosition[] positions = visionPositions.get();
        inputs.poses = new Translation2d[positions.length];
        inputs.tagIds = new int[positions.length];
        for (int i = 0; i < positions.length; i++) {
            VisionPosition position = positions[i];
            inputs.poses[i] = new Translation2d(position.x, position.y);
            inputs.tagIds[i] = position.ID;
        }
    }
}
