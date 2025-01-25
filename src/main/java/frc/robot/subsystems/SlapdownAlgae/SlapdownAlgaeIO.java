package frc.robot.subsystems.SlapdownAlgae;

import org.littletonrobotics.junction.AutoLog;

public interface SlapdownAlgaeIO {
    
    @AutoLog
    public static class SlapdownAlgaeIOInputs {

        //The Volts
        public double pivotappliedVolts = 0.0;
        public double intakeappliedVolts = 0.0;

        //Current
        public double pivotcurrentAmps = 0.0;
        public double intakecurrentAmps = 0.0;

        //Rotating By Degrees
        public double pivotrotationDegrees = 0.0;
        
        //RPM
        public double intakeRPM = 0.0;

    }


    // functions
    public default void updateInputs(SlapdownAlgaeIOInputs inputs) {}

    public default void runVoltage(double volts) {}

}
