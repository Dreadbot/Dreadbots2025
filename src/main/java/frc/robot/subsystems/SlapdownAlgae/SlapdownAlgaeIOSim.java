package frc.robot.subsystems.SlapdownAlgae;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class SlapdownAlgaeIOSim implements SlapdownAlgaeIO {

    private final SingleJointedArmSim SlapdownAlgae;
    private double volts;

    public SlapdownAlgaeIOSim() {
        this.SlapdownAlgae = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            50.0, 
            SingleJointedArmSim.estimateMOI(0.15, Units.lbsToKilograms(10)), 
            0.15, 
            Units.degreesToRadians(-90), 
            Units.degreesToRadians(90), 
            true,
            Units.degreesToRadians(0)
            );

        volts = 0.0;
    }

    @Override
    public void updateInputs(SlapdownAlgaeIOInputs inputs) {
        SlapdownAlgae.update(0.02);

        inputs.pivotappliedVolts = 0.0;
        inputs.intakeappliedVolts = 0.0;
       
        inputs.pivotcurrentAmps = SlapdownAlgae.getCurrentDrawAmps();
        inputs.intakecurrentAmps = SlapdownAlgae.getCurrentDrawAmps();

        inputs.pivotrotationDegrees = Units.radiansToDegrees(SlapdownAlgae.getAngleRads());
    } 

    @Override
    public void runVoltage(double volts) {
        SlapdownAlgae.setInputVoltage(volts);
        this.volts = volts;
    }

}