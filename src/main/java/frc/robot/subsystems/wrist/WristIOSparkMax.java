package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.WristConstants;

public class WristIOSparkMax implements WristIO {
   private final SparkMax wristMotor;
   private DutyCycleEncoder absoluteEncoder;
   private double volts = 0.0;


 public WristIOSparkMax() {
    this.wristMotor = new SparkMax(14, MotorType.kBrushless);
    this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(WristConstants.WRIST_DUTY_CYCLE_ENCODER), 
    WristConstants.WRIST_MAX_ANGLE, WristConstants.WRIST_EXPECTED_ZERO);
    this.volts = 0.0;
    absoluteEncoder.setInverted(true);
    absoluteEncoder.setAssumedFrequency(975.6);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake);
    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 }

  @Override
    public void updateInputs(WristIOInputs inputs) {

        inputs.appliedVolts = 0.0;
       
        //inputs.RPS = (WristMotor.getVelocityRadPerSec());

        inputs.currentAmps = wristMotor.getOutputCurrent();

        inputs.rotationDegrees = (absoluteEncoder.get() - WristConstants.WRIST_ENCODER_OFFSET);

    } 

 @Override
    public void runVoltage(double volts) {
        wristMotor.setVoltage(volts);
        this.volts = volts;
    }
}