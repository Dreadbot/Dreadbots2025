package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{

    public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    public PIDController pid = new PIDController(0, 0, 0);
    public ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);
    public ElevatorIO io;

    public Elevator(ElevatorIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
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
        return run(() -> io.runVoltage(pid.calculate(inputs.positionMeters, goalHeight) 
        + feedforward.calculate(inputs.velocityMPS)));
    }




}
