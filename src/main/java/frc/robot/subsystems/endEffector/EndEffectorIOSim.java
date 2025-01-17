package frc.robot.subsystems.endEffector;

<<<<<<< HEAD
import edu.wpi.first.math.system.LinearSystem;
=======
<<<<<<< HEAD
=======
import static frc.robot.subsystems.drive.DriveConstants.driveGearbox;

>>>>>>> 6b3d42003a1784306096c7e3e65dc759e772bfe0
>>>>>>> 0338e45017128c64f6290fc09c6dcc138ab95af6
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
<<<<<<< HEAD
    
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


       inputs.leftCurrentApps = leftMotorSim.getCurrentDrawAmps();
       inputs.rightCurrentApps = rightMotorSim.getCurrentDrawAmps();
    }

    @Override
    public void runVoltage(double volts) {
        EndEffectorIO.super.runVoltage(volts);
    }

    
    
=======

    private final DCMotorSim leftMotorSim;
    private final DCMotorSim rightMotorSim;
    private final double rollerWheelMOI = 0.5 * Units.lbsToKilograms(0.12) * Units.inchesToMeters(1.5) * Units.inchesToMeters(1.5);
    private double leftVolts;
    private double rightVolts;

    

    public EndEffectorIOSim() {
        this.leftMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 3 * rollerWheelMOI, 1.0),
            DCMotor.getNeoVortex(1)
            
        );
        this.rightMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 3 * rollerWheelMOI, 1.0),
            DCMotor.getNeoVortex(1)
        );
        leftVolts = 0.0;
        rightVolts = 0.0;
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        leftMotorSim.update(0.02);
        rightMotorSim.update(0.02);

        inputs.leftAppliedVolts = 0.0;
        inputs.rightAppliedVolts = 0.0;

        inputs.leftRPM = leftMotorSim.getAngularVelocityRPM();
        inputs.rightRPM = rightMotorSim.getAngularVelocityRPM();

        inputs.leftCurrentAmps = leftMotorSim.getCurrentDrawAmps();
        inputs.rightCurrentAmps = rightMotorSim.getCurrentDrawAmps();

    } 

    @Override
    public void runVoltage(double leftVolts, double rightVolts) {
        leftMotorSim.setInputVoltage(leftVolts);
        rightMotorSim.setInputVoltage(rightVolts);
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }

>>>>>>> 0338e45017128c64f6290fc09c6dcc138ab95af6
}
