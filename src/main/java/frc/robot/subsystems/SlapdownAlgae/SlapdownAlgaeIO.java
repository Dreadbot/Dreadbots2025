package frc.robot.subsystems.slapdownAlgae;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface SlapdownAlgaeIO {
    
    @AutoLog
    public static class SlapdownAlgaeIOInputs {


        //The Volts
        public double pivotAppliedVolts = 0.0;
        public double intakeAppliedVolts = 0.0;

        //Current
        public double pivotCurrentAmps = 0.0;
        public double intakeCurrentAmps = 0.0;

        //Rotating By Degrees
        public double pivotRotationDegrees = 0.0;
        
        //RPM
        public double intakeRPM = 0.0;
        public double absolutePosition = 0.0;

        //Temp
        public double intakeTemperature;
        public double pivotTemperature;

    }


    // functions
    public default void updateInputs(SlapdownAlgaeIOInputs inputs) {};

    public default void runPivotVoltage(double volts) {}

    public default void runIntakeVoltage(double volts) {}

    public default void setIdleMode(IdleMode pivotIdleMode, IdleMode intakeIdleMode) {};

    public default void stopMotors() {};
}
