package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder relativeEncoder; 
    private double volts = 0;
    private double minPosition;
    DigitalInput topLimitSwitch = new DigitalInput(8);
    DigitalInput bottomLimitSwitch = new DigitalInput(9);
    
    public ElevatorIOSparkMax() {
        this.elevatorMotor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        this.relativeEncoder = elevatorMotor.getEncoder();
        this.volts = 0.0;
    }

  @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentAmps = elevatorMotor.getOutputCurrent();
        inputs.voltage = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.positionMeters = relativeEncoder.getPosition();
    } 

 @Override
    public void runVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
        this.volts = volts;
    }

    @Override
    public boolean getBottomLimitSwitch(){
        return bottomLimitSwitch.get();
    }

    @Override
    public boolean getTopLimitSwitch(){
        return topLimitSwitch.get();
    }

    @Override 
    public void setMinPosition(){
        relativeEncoder.setPosition(0);
    }
}