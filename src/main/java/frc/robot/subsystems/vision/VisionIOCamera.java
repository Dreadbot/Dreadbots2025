package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import frc.robot.util.vision.VisionPosition;

public class VisionIOCamera implements VisionIO{
    private StructArraySubscriber<VisionPosition> visionPositions;
    private DoubleSupplier latency;

    public VisionIOCamera(String tableName) {
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        NetworkTable visionTable = ntinst.getTable(tableName);
        this.visionPositions = visionTable.getStructArrayTopic("visionPos", VisionPosition.struct).subscribe(
            new VisionPosition[]{}, 
            PubSubOption.periodic(0.02), 
            PubSubOption.sendAll(true));
        this.latency = visionTable.getDoubleTopic("visionLatency").subscribe(
            0.0, 
            PubSubOption.periodic(0.02), 
            PubSubOption.sendAll(true));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        VisionPosition[] currentPositions = visionPositions.get();
        VisionObservation[] tmp = new VisionObservation[currentPositions.length];
        if(currentPositions.length > 0) {
            for (var i = 0; i < currentPositions.length; i++) {
                var currentPosition = currentPositions[i];
                tmp[i] = new VisionObservation(
                    new Pose2d(
                        currentPosition.x, 
                        currentPosition.y, 
                        Rotation2d.fromRadians(currentPosition.r)), 
                    currentPosition.ID, 
                    visionPositions.getAtomic().timestamp);
            }
        }
        inputs.detections = tmp;
        inputs.visionDelay = latency.getAsDouble();
    }
}
