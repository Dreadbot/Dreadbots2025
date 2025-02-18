package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSparkFlex implements ElevatorIO {
    private final SparkFlex elevatorMotor;
    private final RelativeEncoder relativeEncoder; 
    private final double rotationsToMeters = ElevatorConstants.DRIVING_DRUM_RADIUS * 2 * Math.PI / ElevatorConstants.GEARING;
    private final double metersToRotations = 1 / rotationsToMeters;
    private double volts = 0;
    private double minPosition;
    DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_ID);
    
    public ElevatorIOSparkFlex() {
        this.elevatorMotor = new SparkFlex(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        this.volts = 0.0;
        
        this.relativeEncoder = elevatorMotor.getEncoder();
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .encoder.positionConversionFactor(rotationsToMeters);
        this.elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        inputs.positionMeters = relativeEncoder.getPosition();
    } 

 @Override
    public void runVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
        this.volts = volts;
    }

    @Override
    public boolean getBottomLimitSwitch() {
        return !bottomLimitSwitch.get();
    }

    @Override 
    public void setMinPosition() {
        relativeEncoder.setPosition(ElevatorConstants.MIN_HEIGHT);
    }
}