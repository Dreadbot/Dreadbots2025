package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public VisionObservation[] detections = new VisionObservation[] {};
    }
    public default void updateInputs(VisionIOInputs inputs) {}
    
    public static record VisionObservation(
        Pose2d pose,
        double timestamp,
        int tagId
    ) {}
}
