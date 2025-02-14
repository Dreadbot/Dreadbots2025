package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorIOSparkFlex implements EndEffectorIO{
    private final SparkFlex motor;
    
    public EndEffectorIOSparkFlex(){
        this.motor = new SparkFlex(1, MotorType.kBrushless);
    }

    public void updateInputs(EndEffectorIOInputs inputs){
       inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
       inputs.currentAmps = motor.getOutputCurrent();
    }

    public void runVoltage(double volts){
        motor.setVoltage(volts);
    }
}