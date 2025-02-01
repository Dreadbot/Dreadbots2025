package frc.robot.util.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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
     * Converts Robot Relavtive Tag Cooridnates Pose to world coordinates 
     * @param offset Pose of Tag of from robot coorindate frame
     * @param robotRotation rotation of robot
     * @return Pose converted to world axes
    
    */
    public static Pose2d tagAxisToWorldAxis(Pose3d offset, Rotation2d robotRotation) {
        Rotation3d robotAngle = new Rotation3d(robotRotation.plus(Rotation2d.kPi)); // Rotates angle so it is 180 offset
        Translation3d poseInWorldAxis = offset.getTranslation().rotateBy(robotAngle);
        return new Pose2d(poseInWorldAxis.toTranslation2d(), offset.getRotation().toRotation2d()); // Convert to world axes + ignore Z travel
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
