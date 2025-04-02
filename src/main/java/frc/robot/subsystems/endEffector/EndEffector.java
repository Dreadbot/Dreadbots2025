package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    
    private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
    private EndEffectorIO io;
    private boolean isIntaking = false;
    private boolean hasGamepiece = false;
    private boolean isIntakeCommanded = false;

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
        Logger.recordOutput("EndEffector/HasCoral", hasCoral());
        Logger.recordOutput("EndEffector/IsIntaking", isIntaking);

        if(Math.abs(inputs.RPM) > EndEffectorConstants.INTAKING_THRESHOLD && isIntakeCommanded) { // Use of Math.abs to make logic make sense
            isIntaking = true;
        }
    }

    public Command intake() {
        return startEnd(
            () -> {io.runVoltage(EndEffectorConstants.INTAKE_VOLTAGE); isIntakeCommanded = true; },
            () -> { io.runVoltage(0.0); isIntaking = false; isIntakeCommanded = false;}
        );
    }

    public Command startIntake() {
        return runOnce(
            () -> io.runVoltage(EndEffectorConstants.INTAKE_VOLTAGE)
        );
    }
    public Command startAlgaeIntake() {
        return runOnce(
            () -> io.runVoltage(EndEffectorConstants.ALGAE_INTAKE_VOLTAGE)
        );
    }

    public Command outtake() {
        return startEnd(
            () -> { io.runVoltage(EndEffectorConstants.OUTAKE_VOLTAGE); hasGamepiece = false; },
            () -> io.runVoltage(0.0)
        );
    }
    
    public boolean hasCoral() {
        // if((Math.abs(inputs.RPM) < EndEffectorConstants.CORAL_THRESHOLD_RPM) && isIntaking && inputs.currentAmps > EndEffectorConstants.CORAL_THRESHOLD_AMPS) {
        //     isIntaking = false;
        //     hasGamepiece = true;
        //     return true;
        // }
        return inputs.beamBreakTriggered;
    }

}
