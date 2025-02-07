package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0.0, 0, 0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 1.626, 2.25, 0.15);
    private final ElevatorIO io;
    private double voltage = 0;
    public boolean isZeroed = false;


    private final TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 2));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(Units.inchesToMeters(18), 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(Units.inchesToMeters(18), 0);
    
    public Elevator(ElevatorIO io){
        this.io = io;

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        if (!isZeroed && !DriverStation.isDisabled()) { //isDisabled only needed for sim 
            io.runVoltage(-0.1);
            if (!io.getBottomLimitSwitch()) {
                //If we reach bottom, zero encoder and reset goal;
                io.runVoltage(0);
                isZeroed = true;
                io.setMinPosition();
                setpoint = new TrapezoidProfile.State(inputs.positionMeters, 0);
            }
        } else {
            setpoint = profile.calculate(.02, setpoint, goal);
            voltage = pid.calculate(inputs.positionMeters, setpoint.position)
            + feedforward.calculateWithVelocities(setpoint.velocity, profile.calculate(.02, setpoint, goal).velocity);
            
            io.runVoltage(voltage);
        }
        Logger.recordOutput("Elevator/Goal", goal.position);
        Logger.recordOutput("Elevator/Setpoint", setpoint.position);
        Logger.recordOutput("Elevator/Homed", isZeroed);
    }
    // Look into soft limits: https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/softlimitconfig
    public void setMotorSpeed(double voltage) {
        if (voltage > 0) {
            if (!io.getTopLimitSwitch()) {
                io.runVoltage(0);
            }
            else {
                io.runVoltage(voltage);
            }
        }

        else {
            if (!io.getBottomLimitSwitch()) {
                io.runVoltage(0);
            }
            else {
                io.runVoltage(voltage);
            }
        }
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
        return runOnce(() -> {
            this.goal = new TrapezoidProfile.State(goalHeight, 0);
        });
    }

    public Command setVoltage(double volts){
        return startEnd(
            () -> io.runVoltage(volts),
            () -> io.runVoltage(0.0)
        );
    }

    public Command requestZero() {
        return runOnce(() -> {
            this.isZeroed = false;
        });
    }

    public double getHeight() {
        return inputs.positionMeters;
    }

}
