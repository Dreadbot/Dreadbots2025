package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {


    private final DCMotorSim motorSim;
    private final double rollerWheelMOI = 0.5 * Units.lbsToKilograms(0.12) * Units.inchesToMeters(1.5) * Units.inchesToMeters(1.5);
    public EndEffectorIOSim() {
        this.motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 3 * rollerWheelMOI, 1.0),
            DCMotor.getNeoVortex(1)
            
        );
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        motorSim.update(0.02);

        inputs.appliedVolts = 0.0;

        inputs.RPM = motorSim.getAngularVelocityRPM();

        inputs.currentAmps = motorSim.getCurrentDrawAmps();
      
    } 

    @Override
    public void runVoltage(double volts) {
        motorSim.setInputVoltage(volts);
    }
}
