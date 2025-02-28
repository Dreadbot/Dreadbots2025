package frc.robot.subsystems.slapdownAlgae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlapdownAlgaeConstants;

public class SlapdownAlgae extends SubsystemBase {
    
    
    private final SlapdownAlgaeIOInputsAutoLogged inputs = new SlapdownAlgaeIOInputsAutoLogged();
    private final SlapdownAlgaeIO io;
    public final PIDController pid = new PIDController(0.013, 0.0, 0);
    public final ArmFeedforward feedforward = new ArmFeedforward(0.00, 0.0, 0.018);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(225, 225));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public SlapdownAlgae(SlapdownAlgaeIO io) {
        this.io = io;
    }

    public Command intakeSequence() {
        return Commands.sequence(
            setAngleDegrees(SlapdownAlgaeConstants.INTAKE_ANGLE_DEGREES),
                    Commands.startEnd(
                        () -> io.runIntakeVoltage(SlapdownAlgaeConstants.INTAKE_VOLTAGE),
                        () -> io.runIntakeVoltage(0.0)
                    )
            ).finallyDo(
                () -> {
                    goal = new TrapezoidProfile.State(SlapdownAlgaeConstants.HOLD_ANGLE_DEGREES, 0);
            });
    }

    public Command outtakeSequence() {
        return Commands.sequence(
            setAngleDegrees(SlapdownAlgaeConstants.OUTTAKE_ANGLE_DEGREES),
                Commands.startEnd(
                    () -> io.runIntakeVoltage(SlapdownAlgaeConstants.OUTAKE_VOLTAGE),
                    () -> io.runIntakeVoltage(0.0)
                )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SlapdownIntake", inputs);
        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(inputs.absolutePosition, 0);
            goal = setpoint;
        }
       
        Logger.recordOutput("Slapdown/SetpointPosition", setpoint.position);
        Logger.recordOutput("Slapdown/GoalPosition", goal.position);
        setpoint = profile.calculate(0.02, setpoint, goal);
        io.runPivotVoltage(
            pid.calculate(inputs.absolutePosition, setpoint.position) + 
            feedforward.calculate(inputs.absolutePosition + 90, setpoint.velocity) // use acutal position degrees to make sure that we always apply the correct gravity feed forward.
        ); 
    }

    public Command setAngleDegrees(double angle) {
        return runOnce(
            () -> {
                goal = new TrapezoidProfile.State(angle, 0);
             } );
    }
    public double getAngle() {
        return inputs.pivotRotationDegrees;
    }
}

