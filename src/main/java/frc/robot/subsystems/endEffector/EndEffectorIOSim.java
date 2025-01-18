package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
    
    private final DCMotorSim leftMotorSim;
    private final DCMotorSim rightMotorSim;

    private final double rollerWheelMOI = 0.5 * Units.lbsToKilograms(0.12) * Units.inchesToMeters(1.5);

    public EndEffectorIOSim() {
        this.leftMotorSim = new DCMotorSim (
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 3 * rollerWheelMOI, 1.0),
            DCMotor.getNeoVortex(1)

            );
            this.rightMotorSim = new DCMotorSim (
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 3 * rollerWheelMOI, 1.0),
            DCMotor.getNeoVortex(1)

            );

    }


    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
       leftMotorSim.update(0.02);
       rightMotorSim.update(0.02);

       inputs.leftAppliedVolts = 0;
       inputs.leftRPM = 0;

       inputs.rightAppliedVolts = 0;
       inputs.rightRPM = 0;

       inputs.leftRPM = leftMotorSim.getAngularVelocityRPM();
       inputs.rightRPM = rightMotorSim.getAngularVelocityRPM();


       inputs.leftCurrentAmps = leftMotorSim.getCurrentDrawAmps();
       inputs.rightCurrentAmps = rightMotorSim.getCurrentDrawAmps();
    }

    @Override
    public void runVoltage(double volts) {
        EndEffectorIO.super.runVoltage(volts);
    }

    
    
}
