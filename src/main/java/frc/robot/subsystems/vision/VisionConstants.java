package frc.robot.subsystems.vision;

public final class VisionConstants {
    public static final String frontRightCameraName = "cam0";
    public static final String frontLeftCameraName = "cam1";
    public static final String backCameraName = "cam2";

    public static final double backCameraLinearStdDevs = 0.005;
    public static final double frontLeftCameraLinearStdDevs = 0.005;
    public static final double frontRightCameraLinearStdDevs = 0.005;

    public static final double backCameraAngularStdDevs = 100_000;
    public static final double frontLeftCameraAngularStdDevs = 100_000;
    public static final double frontRightCameraAngularStdDevs = 100_000;
}
