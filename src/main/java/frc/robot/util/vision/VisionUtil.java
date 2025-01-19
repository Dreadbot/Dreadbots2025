package frc.robot.util.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionUtil {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);


    public static Pose2d getApriltagPose(int tagId) {
        return FIELD_LAYOUT.getTagPose(tagId).get().toPose2d();
    }

}
