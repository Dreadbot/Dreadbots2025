package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    
    private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
    private EndEffectorIO io;
    private boolean isIntaking = false;

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
        if(inputs.RPM > EndEffectorConstants.CORAL_THRESHOLD) {
            isIntaking = true;
        }
    }

    public Command intake() {
        return startEnd(
            () -> io.runVoltage(EndEffectorConstants.INTAKE_VOLTAGE),
            () -> { io.runVoltage(0.0); isIntaking = false; }
        );
    }
    public Command outtake() {
        return startEnd(
            () -> io.runVoltage(EndEffectorConstants.OUTAKE_VOLTAGE),
            () -> io.runVoltage(0.0)
        );
    }
    
    public boolean hasGamePiece() {
        return inputs.RPM < EndEffectorConstants.CORAL_THRESHOLD;
    }

    public boolean hasCoral() {
        if((inputs.RPM < EndEffectorConstants.CORAL_THRESHOLD) && isIntaking) {
            isIntaking = false;
            return true;
        }
        return false;
    }

}
