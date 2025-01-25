package frc.robot.subsystems.SlapdownAlgae;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
        public double absolutePosition;

        //Temp
        //public Object intakeTemperature;
        //public Object pivotTemperature;

    }


    // functions
    public default void updateInputs(SlapdownAlgaeIOInputs inputs) {};


    public default void close(SlapdownAlgaeIOInputs inputs) {};


    public default void runVoltage(double voltage) {};


    public default void setIdleMode(IdleMode idleMode) {};

  
    public default void close(){};
        
   
    public default void stopMotors() {};
}
