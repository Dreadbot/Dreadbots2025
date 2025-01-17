package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

public interface EndEffectorIO {

    @AutoLog
    public static class EndEffectorIOInputs {
        public double leftRPM = 0.0;
        public double rightRPM = 0.0;


        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;

        public double leftCurrentApps = 0.0;
        public double rightCurrentApps = 0.0;

        public double speed = 0.0;
        public double current = 0.0;
        public double voltage = 0.0;



    }
    public default void updateInputs(EndEffectorIOInputs inputs) {}
    public default void runVoltage(double volts) {};
    public default void changeCurrentLimit(double current) {};
    
}