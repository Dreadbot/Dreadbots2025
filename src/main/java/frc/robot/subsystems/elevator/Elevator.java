package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController pid = new PIDController(.5, 0, 0);

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0.5, 0.15);
    //private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 1.626, 2.25, 0.15);
    private final ElevatorIO io;
    private double voltage = 0;
    public boolean isZeroed = false;
    private double joystickAxis; 


    private final TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 2));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(Units.inchesToMeters(18), 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(Units.inchesToMeters(18), 0);
    private State desiredElevatorState;
    public double joystickOverride;
    public DoubleSupplier joystickOverride1;

    public Elevator(ElevatorIO io){
        this.io = io;
        this.joystickOverride = 0.0;
        desiredElevatorState = new State(0, 0);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

            //If we reach bottom, zero encoder and reset goal;
        if (!isZeroed && !DriverStation.isDisabled()) {  // isDisabled only needed for sim 
           // io.runVoltage(-1);
            voltage = -1;
            if (!io.getBottomLimitSwitch()) {
                voltage = 0;
                isZeroed = true;
                io.setMinPosition();
                setpoint = new TrapezoidProfile.State(Units.inchesToMeters(18), 0);
            }
           // return;
        } else {
            TrapezoidProfile.State currentState = setpoint;
            setpoint = profile.calculate(.02, setpoint, goal);
            double pidValue = pid.calculate(inputs.positionMeters, setpoint.position);
            double feedforwardValue = feedforward.calculateWithVelocities(currentState.velocity, setpoint.velocity);
            voltage = pidValue + feedforwardValue;

            Logger.recordOutput("Elevator/Feedforward", feedforwardValue);
            Logger.recordOutput("Elevator/PID", pidValue);
    
           // System.out.println("PID: " + pidValue + " Feed " + feedforwardValue);
           // io.runVoltage(voltage);
           // System.out.println(" Voltage: " + voltage);
        }

         if (Math.abs(joystickOverride) > 0.08) {
            
            // this.desiredElevatorState = new State(
            //     MathUtil.clamp(
            //         this.desiredElevatorState.position + joystickOverride * ElevatorConstants.ELEVATOR_JOYSTICK_SLEW_VALUE,
            //         0.000,
            //         ElevatorConstants.MAX_HEIGHT
            //     ),
            //     0
            // );
            voltage = joystickOverride * 5;
        
        }

        setMotorSpeed(voltage);
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

    public Command setJoystickOverride(DoubleSupplier joystickValue) {
        return runOnce (
            () -> {
                joystickOverride = joystickValue.getAsDouble();
                System.out.println(joystickValue.getAsDouble());
            }
        );
    }

    // public void setJoystick(DoubleSupplier joystickAxis) {
    //     this.joystickOverride1 = joystickAxis;
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

    public void init() {
        isZeroed = false;
        voltage = 0;
    }
}   
