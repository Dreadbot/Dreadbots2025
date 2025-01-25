package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.WristConstants;

public class WristIOSparkFlex implements WristIO {
   private final SparkFlex WristMotor;
   private DutyCycleEncoder absoluteEncoder;
   private double volts = 0.0;


 public WristIOSparkFlex() {
    this.WristMotor = new SparkFlex(2, MotorType.kBrushless);
    this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(WristConstants.WRIST_DUTY_CYCLE_ENCODER), 
    WristConstants.WRIST_MAX_ANGLE, WristConstants.WRIST_ENCODER_OFFSET);
    this.volts = 0.0;
 }

  @Override
    public void updateInputs(WristIOInputs inputs) {

        inputs.appliedVolts = 0.0;
       
        //inputs.RPS = (WristMotor.getVelocityRadPerSec());

        inputs.currentAmps = WristMotor.getOutputCurrent();

        //inputs.rotationDegrees = Units.radiansToDegrees(absoluteEncoder.);

        //inputs.rotationDegrees = absoluteEncoder.get();

    } 

 @Override
    public void runVoltage(double volts) {
        WristMotor.setVoltage(volts);
        this.volts = volts;
    }
}