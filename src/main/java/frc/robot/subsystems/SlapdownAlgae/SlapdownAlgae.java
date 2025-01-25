package frc.robot.subsystems.SlapdownAlgae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlapdownAlgaeConstants;

public class SlapdownAlgae extends SubsystemBase {
    
    private SlapdownAlgaeIOInputsAutoLogged inputs = new SlapdownAlgaeIOInputsAutoLogged();
    private SlapdownAlgaeIO io;
    public PIDController pid = new PIDController(1.5, 0.005, 0);
    public ArmFeedforward feedforward = new ArmFeedforward(0.0, 1, 0.25);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(180, 180));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private double goalAngle;

    public SlapdownAlgae(SlapdownAlgaeIO io) {
        this.io = io;
    }

    public Command intake() {
        return startEnd(
        ()-> io.runVoltage(SlapdownAlgaeConstants.INTAKE_VOLTAGE),
        ()->io.runVoltage(0.0)

        );
    }

    public Command outtake() {
        return startEnd(
        ()-> io.runVoltage(SlapdownAlgaeConstants.OUTAKE_VOLTAGE),
        ()->io.runVoltage(0.0)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SlapdownAlgaePivot", inputs);
        goal = new TrapezoidProfile.State(goalAngle, 0);
        Logger.recordOutput("AlgaeSetpointPosition", setpoint.position);
        Logger.recordOutput("AlgaeGoalAngle", goalAngle);
        Logger.recordOutput("AlgaeGoalPosition", goal.position);
        setpoint = profile.calculate(0.02, setpoint, goal);
        io.runVoltage(pid.calculate(inputs.pivotrotationDegrees, setpoint.position) + feedforward.calculate(Units.degreesToRadians(setpoint.position) ,setpoint.velocity));
    }

    public Command setAngleDegrees(double angle) {
        
        return run(
            () -> {
                goalAngle = angle;
             } );
    }
}
