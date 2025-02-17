package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.WristConstants;

public class WristIOSparkMax implements WristIO {
   private final SparkMax WristMotor;
   private DutyCycleEncoder absoluteEncoder;
   private double volts = 0.0;


 public WristIOSparkMax() {
    this.WristMotor = new SparkMax(1, MotorType.kBrushless);
    this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(WristConstants.WRIST_DUTY_CYCLE_ENCODER), 
    WristConstants.WRIST_MAX_ANGLE, WristConstants.WRIST_EXPECTED_ZERO);
    absoluteEncoder.setInverted(true);
    absoluteEncoder.setAssumedFrequency(WristConstants.ENCODER_FREQUENCY);
    this.volts = 0.0;
   
 }

  @Override
    public void updateInputs(WristIOInputs inputs) {

        inputs.appliedVolts = 0.0;
       
        //inputs.RPS = (WristMotor.getVelocityRadPerSec());

        inputs.currentAmps = WristMotor.getOutputCurrent();

        inputs.rotationDegrees = absoluteEncoder.get() - WristConstants.WRIST_ENCODER_OFFSET; //offset

    } 

 @Override
    public void runVoltage(double volts) {
        WristMotor.setVoltage(volts);
        this.volts = volts;
    }
}