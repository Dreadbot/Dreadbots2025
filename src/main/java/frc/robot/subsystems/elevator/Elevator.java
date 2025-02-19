package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0, 0, 0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0.5, 0.1);
    private final ElevatorIO io;
    private final TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 2));
        private TrapezoidProfile.State goal = new TrapezoidProfile.State(Units.inchesToMeters(18), 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(Units.inchesToMeters(70), 0);
    public double joystickOverride;
    public boolean isZeroed = false;
    public DoubleSupplier joystickOverride1;
    private double voltage = 0;

    public Elevator(ElevatorIO io){
        this.io = io;
        this.joystickOverride = 0.0;
        new State(0, 0);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    
        System.out.println("Driver Station: " + DriverStation.isDisabled());

        //If we reach bottom, zero encoder and reset goal;
        System.out.println("Joystick Periodic : " + joystickOverride);
        if (!isZeroed && !DriverStation.isDisabled()) {  //isDisabled only needed for sim 
           // io.runVoltage(-1);
            voltage = -1;
            System.out.println("Zeroed: " + isZeroed);
            if (!io.getBottomLimitSwitch()) {
                voltage = 0;
                isZeroed = true;
                io.setMinPosition();
                setpoint = new TrapezoidProfile.State(Units.inchesToMeters(18), 0);
                //System.out.println("Zeroed: " + isZeroed);
            }
        } else {
            TrapezoidProfile.State currentState = setpoint;
            setpoint = profile.calculate(.02, setpoint, goal);
            System.out.println("Set: " + setpoint.position + " Goal: " + goal.position + " Position " + inputs.positionMeters);
            double pidValue = pid.calculate(inputs.positionMeters, setpoint.position);
            double feedforwardValue = feedforward.calculateWithVelocities(currentState.velocity, setpoint.velocity);
            voltage = pidValue + feedforwardValue;
            Logger.recordOutput("Feedforward", feedforwardValue);
            Logger.recordOutput("PID", pidValue);
            Logger.recordOutput("Setpoint", setpoint.position);
    
           // System.out.println("PID: " + pidValue + " Feed " + feedforwardValue);
           // io.runVoltage(voltage);
           // System.out.println(" Voltage: " + voltage);
        }

         if (Math.abs(joystickOverride) > 0.08) {
            
            setpoint = new State(inputs.positionMeters, 0);
//                MathUtil.clamp(
  //                  setpoint.position + joystickOverride * ElevatorConstants.ELEVATOR_JOYSTICK_SLEW_VALUE,
    //                0.000,
      //              ElevatorConstants.MAX_HEIGHT
        //        ),
          //      0
            //);
            goal = setpoint;

            voltage = joystickOverride * 2.5;
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
                System.out.println("Joystick Override: " + joystickValue.getAsDouble());
            }
        );
    }

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
