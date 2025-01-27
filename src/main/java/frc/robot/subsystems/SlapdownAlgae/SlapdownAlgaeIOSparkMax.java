package frc.robot.subsystems.SlapdownAlgae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.SlapdownAlgaeConstants;


public class SlapdownAlgaeIOSparkMax implements SlapdownAlgaeIO {

    private final CANSparkMax inOutTakeMotor;
    private final CANSparkMax pivotMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public SlapdownAlgaeIOSparkMax() {
        this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(SlapdownAlgaeConstants.SLAPDOWNALGAE_DUTY_CYCLE_ENCODER)); //Update code with the 0 and max angle

        this.inOutTakeMotor = new CANSparkMax();
        this.pivotMotor = new CANSparkMax();
    }

        @Override
        public void updateInputs(SlapdownAlgaeIOInputs inputs) {
            inputs.absolutePosition = absoluteEncoder.get();

            inputs.intakeappliedVolts = inOutTakeMotor.getAppliedOutput() * inOutTakeMotor.getBusVoltage();
            inputs.intakecurrentAmps = inOutTakeMotor.getOutputCurrent();
            //inputs.intakeTemperature = inOutTakeMotor.getMotorTemperature();

            inputs.pivotappliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
            inputs.pivotcurrentAmps = pivotMotor.getOutputCurrent();
           // inputs.pivotTemperature = pivotMotor.getMotorTemperature();
        }

         @Override
        public void runVoltage(double voltage) {
            inOutTakeMotor.setVoltage(voltage);
        }

        @Override
        public void setIdleMode(IdleMode idleMode) {
            inOutTakeMotor.setIdleMode(idleMode);
            pivotMotor.setIdleMode(idleMode);
        }

        @Override
        public void stopMotors() {
            inOutTakeMotor.setVoltage(0);
            pivotMotor.setVoltage(0);
        }
    }
