package frc.robot.util.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionUtil {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    /**
     * 
     * @param tagId ID of tag
     * @return Pose3d of apriltag position
     */
    public static Pose3d getApriltagPose(int tagId) {
        return FIELD_LAYOUT.getTagPose(tagId).get();
    }

    /**
     * Converts Tag Pose to world coordinates 
     * @param offset Pose of Tag of from robot coorindate frame
     * @param tagPose Pose of tag in world frame
     * @return Pose converted to world axes
    
    */
    public static Pose2d tagAxisToWorldAxis(Pose3d offset, Pose3d tagPose) {
        return new Pose2d(offset.getTranslation().rotateAround(tagPose.getTranslation(), tagPose.getRotation()).toTranslation2d(), offset.getRotation().toRotation2d()); // Convert to world axes + ignore Z travel
    }

    /**
     * Returns whether the ID is not a reef ID
     * @param id ID of tag
     * @return Indicates whether ID is not reef ID or is
     */

    public static boolean isNotReefId(int id) {
        return id == 1
        || id == 2
        || id == 3
        || id == 4
        || id == 5
        || id == 12
        || id == 13
        || id == 14
        || id == 15
        || id == 16;
    }
    /**
     * Finds global pose given the world axes offset and ID of the tag
     * @param offset World axes offset
     * @param tagId ID of tag
     * 
     */
    public static Pose2d calculatePoseFromTagOffset(Pose2d offset, int tagId) {
        return new Pose2d(getApriltagPose(tagId).toPose2d().getTranslation().minus(offset.getTranslation()), offset.getRotation());
    }
}
