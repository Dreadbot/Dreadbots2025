package frc.robot.subsystems.slapdownAlgae;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.SlapdownAlgaeConstants;


public class SlapdownAlgaeIOSim implements SlapdownAlgaeIO {

    private final SingleJointedArmSim slapdownAlgae;
    private final DCMotorSim intakeMotor;

    private double pivotVolts;
    private double intakeVolts;


    public SlapdownAlgaeIOSim() {
        this.slapdownAlgae = new SingleJointedArmSim(
            DCMotor.getNeoVortex(1), 
            50.0, 
            SlapdownAlgaeConstants.SIM_PIVOT_MOI, 
            Units.inchesToMeters(16), 
            Units.degreesToRadians(-20), 
            Units.degreesToRadians(90), 
            true,
            Units.degreesToRadians(0)
            );
        this.intakeMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), SlapdownAlgaeConstants.SIM_INTAKE_MOI, 2), 
            DCMotor.getNEO(1)
        );
        pivotVolts = 0.0;
        intakeVolts = 0.0;

    }

    @Override
    public void updateInputs(SlapdownAlgaeIOInputs inputs) {
        slapdownAlgae.update(0.02);
        intakeMotor.update(0.02);

        inputs.pivotAppliedVolts = pivotVolts;
        inputs.intakeAppliedVolts = intakeVolts;
       
        inputs.pivotCurrentAmps = slapdownAlgae.getCurrentDrawAmps();
        inputs.intakeCurrentAmps = intakeMotor.getCurrentDrawAmps();

        inputs.pivotRotationDegrees = Units.radiansToDegrees(slapdownAlgae.getAngleRads());

        inputs.intakeRPM = intakeMotor.getAngularVelocityRPM();
    } 

    @Override
    public void runPivotVoltage(double volts) {
        slapdownAlgae.setInputVoltage(volts);
        this.pivotVolts = volts;
    }
    @Override
    public void runIntakeVoltage(double volts) {
        intakeMotor.setInputVoltage(volts);
        this.intakeVolts = volts;
    }

}