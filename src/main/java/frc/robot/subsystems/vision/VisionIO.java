package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Translation2d[] poses;
        public int[] tagIds;
    }
    public default void updateInputs(VisionIOInputs inputs) {}
    
}
