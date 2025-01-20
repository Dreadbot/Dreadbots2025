package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.TimestampedObject;
import frc.robot.util.vision.VisionPosition;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public VisionObservation[] detections;
    }
    public default void updateInputs(VisionIOInputs inputs) {}
    
    public static record VisionObservation(
        Pose2d pose,
        double timestamp,
        int tagId
    ) {}
}
