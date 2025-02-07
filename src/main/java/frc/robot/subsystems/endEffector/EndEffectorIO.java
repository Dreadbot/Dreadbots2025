package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public static class EndEffectorIOInputs {
        public double RPM = 0.0;

        public double appliedVolts = 0.0;

        public double currentAmps = 0.0;
    }
    public default void updateInputs(EndEffectorIOInputs inputs) {}

    public default void runVoltage(double volts) {};
    
    public default void changeCurrentLimit(double current) {};
    
}