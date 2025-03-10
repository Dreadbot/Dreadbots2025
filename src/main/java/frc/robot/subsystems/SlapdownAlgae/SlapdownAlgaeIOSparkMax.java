package frc.robot.subsystems.slapdownAlgae;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
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

    private final SparkBase intakeMotor;
    private final SparkBase pivotMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public SlapdownAlgaeIOSparkMax() {
        this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(SlapdownAlgaeConstants.SLAPDOWNALGAE_DUTY_CYCLE_ENCODER), 360, 0); //Update code with the 0 and max angle
        absoluteEncoder.setAssumedFrequency(SlapdownAlgaeConstants.ENCODER_FREQUENCY);
        this.intakeMotor = new SparkMax(SlapdownAlgaeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.pivotMotor = new SparkFlex(SlapdownAlgaeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        intakeConfig
            .idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        @Override
        public void updateInputs(SlapdownAlgaeIOInputs inputs) {
            inputs.absolutePosition = absoluteEncoder.get() - SlapdownAlgaeConstants.ENCODER_OFFSET;
            inputs.intakeRPM = intakeMotor.getEncoder().getVelocity();

            inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
            inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
            inputs.intakeTemperature = intakeMotor.getMotorTemperature();

            inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
            inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
            inputs.pivotTemperature = pivotMotor.getMotorTemperature();
        }

        @Override
        public void runIntakeVoltage(double voltage) {
            intakeMotor.setVoltage(voltage);
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
            intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        @Override
        public void stopMotors() {
            intakeMotor.setVoltage(0);
            intakeMotor.stopMotor();
            pivotMotor.setVoltage(0);
            pivotMotor.stopMotor();
        }

        @Override
        public void runPivotVoltage(double voltage){
            pivotMotor.setVoltage(voltage);
        }
    }
