package frc.robot.subsystems.SlapdownAlgae;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.SlapdownAlgaeConstants;


public class SlapdownAlgaeIOSparkMax implements SlapdownAlgaeIO {

    //private final SparkBase inOutTakeMotor;
    private final SparkBase pivotMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public SlapdownAlgaeIOSparkMax() {
        this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(SlapdownAlgaeConstants.SLAPDOWNALGAE_DUTY_CYCLE_ENCODER)); //Update code with the 0 and max angle

        //this.inOutTakeMotor = new SparkMax(0, MotorType.kBrushless);
        this.pivotMotor = new SparkMax(1, MotorType.kBrushless);
    }

        @Override
        public void updateInputs(SlapdownAlgaeIOInputs inputs) {
            inputs.absolutePosition = absoluteEncoder.get();

            // inputs.intakeAppliedVolts = inOutTakeMotor.getAppliedOutput() * inOutTakeMotor.getBusVoltage();
            // inputs.intakeCurrentAmps = inOutTakeMotor.getOutputCurrent();
            //inputs.intakeTemperature = inOutTakeMotor.getMotorTemperature();

            inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
            inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
           // inputs.pivotTemperature = pivotMotor.getMotorTemperature();
        }

        @Override
        public void runIntakeVoltage(double voltage) {
       //     inOutTakeMotor.setVoltage(voltage);
        }

        @Override
        public void runPivotVoltage(double voltage) {
            pivotMotor.setVoltage(voltage);
        }

        @Override
        public void setIdleMode(IdleMode pivotIdleMode, IdleMode intakeIdleMode) {
            SparkMaxConfig pivotConfig = new SparkMaxConfig();
            pivotConfig.idleMode(pivotIdleMode);
            SparkMaxConfig intakeConfig = new SparkMaxConfig();
            intakeConfig.idleMode(intakeIdleMode);
            /* 
            * Don't reset parameters + don't save this config if reboot hapens. 
            * This is ony if we need to switch out of break mode for some reason
            */ 
           // inOutTakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        @Override
        public void stopMotors() {
            // inOutTakeMotor.setVoltage(0);
            // inOutTakeMotor.stopMotor();
            pivotMotor.setVoltage(0);
            pivotMotor.stopMotor();
        }
    }
