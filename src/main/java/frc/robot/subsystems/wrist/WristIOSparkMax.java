package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
<<<<<<< HEAD
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
=======
import com.revrobotics.spark.SparkMax;
>>>>>>> origin/choreo-autos

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.WristConstants;

public class WristIOSparkMax implements WristIO {
<<<<<<< HEAD
   private final SparkMax wristMotor;
=======
   private final SparkMax WristMotor;
>>>>>>> origin/choreo-autos
   private DutyCycleEncoder absoluteEncoder;
   private double volts = 0.0;


 public WristIOSparkMax() {
<<<<<<< HEAD
    this.wristMotor = new SparkMax(14, MotorType.kBrushless);
=======
    this.WristMotor = new SparkMax(1, MotorType.kBrushless);
>>>>>>> origin/choreo-autos
    this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(WristConstants.WRIST_DUTY_CYCLE_ENCODER), 
    WristConstants.WRIST_MAX_ANGLE, WristConstants.WRIST_EXPECTED_ZERO);
    this.volts = 0.0;
    absoluteEncoder.setInverted(true);
<<<<<<< HEAD
    absoluteEncoder.setAssumedFrequency(975.6);
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
=======
>>>>>>> origin/choreo-autos
 }

  @Override
    public void updateInputs(WristIOInputs inputs) {

<<<<<<< HEAD
        inputs.appliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
       
        //inputs.RPS = (WristMotor.getVelocityRadPerSec());

        inputs.currentAmps = wristMotor.getOutputCurrent();

        inputs.rotationDegrees = (absoluteEncoder.get() - WristConstants.WRIST_ENCODER_OFFSET);
=======
        inputs.appliedVolts = 0.0;
       
        //inputs.RPS = (WristMotor.getVelocityRadPerSec());

        inputs.currentAmps = WristMotor.getOutputCurrent();

        inputs.rotationDegrees = absoluteEncoder.get() - WristConstants.WRIST_ENCODER_OFFSET;
>>>>>>> origin/choreo-autos

    } 

 @Override
    public void runVoltage(double volts) {
<<<<<<< HEAD
        wristMotor.setVoltage(volts);
=======
        WristMotor.setVoltage(volts);
>>>>>>> origin/choreo-autos
        this.volts = volts;
    }
}