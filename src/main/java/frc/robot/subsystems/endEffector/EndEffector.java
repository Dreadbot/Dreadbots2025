package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    
    private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
    private EndEffectorIO io;

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }

    public Command intake() {
        return startEnd(
            () -> io.runVoltage(EndEffectorConstants.INTAKE_VOLTAGE),
            () -> io.runVoltage(0.0)
        );
    }
    public Command outtake() {
        return startEnd(
            () -> io.runVoltage(EndEffectorConstants.OUTAKE_VOLTAGE),
            () -> io.runVoltage(0.0)
        );
    }
    
    public boolean hasGamePiece() {
        return inputs.RPM < EndEffectorConstants.INTAKE_THRESHOLD;
    }

}
