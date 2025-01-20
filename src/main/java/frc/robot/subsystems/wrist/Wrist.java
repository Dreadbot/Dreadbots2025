package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private WristIO io;
    public PIDController pid = new PIDController(0.1, 0.001, 0);
    public ArmFeedforward feedforward = new ArmFeedforward(0.1, 0.001, 0);
    

    public Wrist(WristIO io) {
        this.io = io;
    } 

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    public Command setAngleDegrees(double goalAngle) {
        return startEnd(
            () -> io.runVoltage(pid.calculate(inputs.rotationDegrees, goalAngle) + 
            feedforward.calculate(Units.degreesToRadians(goalAngle), 1)), 
            () -> io.runVoltage(0.0)
        );
    }
}
