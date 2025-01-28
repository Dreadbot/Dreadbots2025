package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0, 0, 0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 1.263, 2.91666667, 0.25);
    private final ElevatorIO io;
    private double goalHeight = 0;
    private double voltage = 0;

    private final TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 2));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    
    public Elevator(ElevatorIO io){
        this.io = io;

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        goal = new TrapezoidProfile.State(goalHeight, 0);
        setpoint = profile.calculate(.02, setpoint, goal);
        voltage = pid.calculate(inputs.positionMeters, setpoint.position)
        + feedforward.calculateWithVelocities(setpoint.velocity, profile.calculate(.02, setpoint, goal).velocity);
        Logger.recordOutput("Elevator/Goal", goal.position);
        Logger.recordOutput("Elevator/Setpoint", setpoint.position);

        io.runVoltage(voltage);
    }

    // public Command rise(){
    //     return startEnd(
    //     () -> elevatorIO.runVoltage(ElevatorConstants.RISE_VOLTAGE), 
    //     () -> elevatorIO.runVoltage(0)
    //     );
    // }
    // public Command drop(){
    //     return startEnd(
    //     () -> elevatorIO.runVoltage(ElevatorConstants.DROP_VOLTAGE), 
    //     () -> elevatorIO.runVoltage(0)
    //     );
    // }

    public Command riseTo(double goalHeight){
        return run(() -> {
            this.goalHeight = goalHeight;
        });
    }

    public double getHeight() {
        return inputs.positionMeters;
    }

}
