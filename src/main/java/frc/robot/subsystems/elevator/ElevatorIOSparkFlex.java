package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSparkFlex implements ElevatorIO {
    private final SparkFlex elevatorMotor;
    private final RelativeEncoder relativeEncoder; 
    private final double rotationsToMeters = ElevatorConstants.DRIVING_DRUM_RADIUS * 2 * Math.PI / ElevatorConstants.GEARING;
    private final double metersToRotations = 1 / rotationsToMeters;
    DigitalInput topLimitSwitch = new DigitalInput(8);
    DigitalInput bottomLimitSwitch = new DigitalInput(9);
    
    public ElevatorIOSparkFlex() {
        this.elevatorMotor = new SparkFlex(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        this.relativeEncoder = elevatorMotor.getEncoder();

        // old code 
        // EncoderConfig encoderConf = new EncoderConfig();
        // encoderConf.positionConversionFactor(ElevatorConstants.DRIVING_DRUM_RADIUS * 2 * Math.PI / ElevatorConstants.GEARING);
        // SparkBaseConfig sparkConfig = new SparkFlexConfig();
        // sparkConfig.apply(encoderConf);
    }

  @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentAmps = elevatorMotor.getOutputCurrent();
        inputs.voltage = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.positionMeters = relativeEncoder.getPosition() * rotationsToMeters;
    } 

 @Override
    public void runVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
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
        relativeEncoder.setPosition(Units.inchesToMeters(18) * metersToRotations);
    }
}