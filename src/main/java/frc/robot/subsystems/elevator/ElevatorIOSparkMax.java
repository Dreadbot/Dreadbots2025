package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder relativeEncoder; 
    private double volts = 0;
    
    public ElevatorIOSparkMax() {
    this.elevatorMotor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
    this.relativeEncoder = elevatorMotor.getEncoder();
    this.volts = 0.0;
 }

  @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // inputs.positionMeters = relativeEncoder.getPosition() * ElevatorConstants.METERS_PER_ROTATION; 
        // inputs.velocityMPS = relativeEncoder.getVelocity() * ElevatorConstants.METERS_PER_ROTATION / 60.0; 
        inputs.currentAmps = elevatorMotor.getOutputCurrent();
        inputs.voltage = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.currentAmps = elevatorMotor.getOutputCurrent();
    } 

 @Override
    public void runVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
        this.volts = volts;
    }
}