package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    public PIDController pid = new PIDController(3, 0, 0);
    public ElevatorFeedforward feedforward = new ElevatorFeedforward(2, 2, 2);
    public ElevatorIO io;
    public double goalHeight = 0;
    public double voltage = 0;

    private final TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(.3, 0.3));
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
        + feedforward.calculate(setpoint.velocity);
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

}
