package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOSparkFlex implements EndEffectorIO{
    private final SparkFlex motor;
    
    public EndEffectorIOSparkFlex(){
        this.motor = new SparkFlex(EndEffectorConstants.MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(EndEffectorIOInputs inputs){
       inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
       inputs.currentAmps = motor.getOutputCurrent();
       inputs.RPM = motor.getEncoder().getVelocity();
    }

    public void runVoltage(double volts){
        motor.setVoltage(volts);
    }
}