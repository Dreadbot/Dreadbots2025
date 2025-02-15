package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.WristConstants;

public class WristIOSparkMax implements WristIO {
   private final SparkMax wristMotor;
   private SparkMaxConfig config;
   private DutyCycleEncoder absoluteEncoder;
   
   public WristIOSparkMax() {
    this.wristMotor = new SparkMax(1, MotorType.kBrushless);
    this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(WristConstants.WRIST_DUTY_CYCLE_ENCODER), 
    WristConstants.WRIST_MAX_ANGLE, WristConstants.WRIST_ENCODER_OFFSET);
    this.config = new SparkMaxConfig();
    this.config.encoder.positionConversionFactor(WristConstants.GEAR_REDUCTION * 360);
    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 }

  @Override
    public void updateInputs(WristIOInputs inputs) {

        inputs.appliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
       
        inputs.RPS = wristMotor.get();

        inputs.currentAmps = wristMotor.getOutputCurrent();

        inputs.rotationDegrees = wristMotor.getEncoder().getPosition();

    } 

 @Override
    public void runVoltage(double volts) {
        wristMotor.setVoltage(volts);
    }
}