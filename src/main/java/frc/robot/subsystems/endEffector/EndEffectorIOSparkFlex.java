package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorIOSparkFlex implements EndEffectorIO{
    private final SparkFlex motor;
    private double volts;

    public EndEffectorIOSparkFlex(){
        this.motor = new SparkFlex(16, MotorType.kBrushless);
        this.volts = 0.0;
    }

    public void updateInputs(EndEffectorIOInputs inputs){
       inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
       inputs.currentAmps = motor.getOutputCurrent();
       inputs.RPM = motor.getEncoder().getVelocity();
    }

    public void runVoltage(double volts){
        motor.setVoltage(volts);
        this.volts = volts;
    }
}