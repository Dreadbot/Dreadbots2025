package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SlapdownAlgaeConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax elevatorMotor;
    private final DutyCycleEncoder absoluteEncoder; 
    private double volts = 0;
    
    public ElevatorIOSparkMax() {
    this.elevatorMotor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
    this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(ElevatorConstants.DUTY_CYCLE_ENCODER)); 

    this.volts = 0.0;
 }

  @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentAmps = elevatorMotor.getOutputCurrent();
        inputs.voltage = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.currentAmps = elevatorMotor.getOutputCurrent();
        inputs.positionMeters = absoluteEncoder.get();

    } 

 @Override
    public void runVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
        this.volts = volts;
    }
}