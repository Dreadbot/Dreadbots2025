package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public static class EndEffectorIOInputs {
        public double leftRPM = 0.0; 
        public double rightRPM = 0.0; 
        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    // functions
    public default void updateInputs(EndEffectorIOInputs inputs) {}

    public default void runVoltage(double leftVolts, double rightVolts) {}

    public default void changeCurrentLimit(double current) {}

}
