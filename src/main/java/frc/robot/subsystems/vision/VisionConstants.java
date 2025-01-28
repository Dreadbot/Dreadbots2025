package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {
    public static final String FRONT_CAMERA_NAME = "azathoth";
    public static final Matrix<N3, N1> STD_DEV = VecBuilder.fill(0.05, 0.05, 100_000); // don't trust rotation;
}
