package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    
    @AutoLog
    public static class WristIOInputs {
        public double RPS = 0.0; 
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double rotationDegrees = 0.0;
    }

    // functions
    public default void updateInputs(WristIOInputs inputs) {}

    public default void runVoltage(double Volts) {}



}
