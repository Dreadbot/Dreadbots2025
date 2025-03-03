package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Superstructure.SuperstructureState;


public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0.02, 0, 0);
    

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.09, 0.24, 5.8, 0.0);
    private final ElevatorIO io;
    private double voltage = 0;
    public boolean isZeroed = false; 

    private final TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(2.5, 1.5)); // Slow to start
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(ElevatorConstants.MIN_HEIGHT, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(ElevatorConstants.MIN_HEIGHT, 0);
    public DoubleSupplier joystickOverride;

    public Elevator(ElevatorIO io){
        this.io = io;
        this.joystickOverride = () -> 0.0;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if(DriverStation.isDisabled()) { //If we are disabled, reset setpoint to elevator position
            setpoint = new TrapezoidProfile.State(inputs.positionMeters, 0);
            goal = setpoint;
        }
        //If we reach bottom, zero encoder and reset goal;
        if (!isZeroed) {
            voltage = ElevatorConstants.ZEROING_VOLTAGE;
            if (io.getBottomLimitSwitch()) {
                voltage = 0;
                isZeroed = true;
                io.setMinPosition();
                setpoint = new TrapezoidProfile.State(ElevatorConstants.MIN_HEIGHT, 0);
                goal = setpoint;
            }
        } else { 
            TrapezoidProfile.State currentState = setpoint;
            setpoint = profile.calculate(.02, setpoint, goal);
            double pidValue = pid.calculate(inputs.positionMeters, setpoint.position);
            double feedforwardValue = feedforward.calculateWithVelocities(currentState.velocity, setpoint.velocity);
            voltage = pidValue + feedforwardValue;
            Logger.recordOutput("Elevator/Feedforward", feedforwardValue);
            Logger.recordOutput("Elevator/PID", pidValue);
            Logger.recordOutput("Elevator/Setpoint", setpoint.position);
        }

         if (Math.abs(joystickOverride.getAsDouble()) > 0.10) {
            
            setpoint = new State(inputs.positionMeters, 0);
//                MathUtil.clamp(
  //                  setpoint.position + joystickOverride * ElevatorConstants.ELEVATOR_JOYSTICK_SLEW_VALUE,
    //                0.000,
      //              ElevatorConstants.MAX_HEIGHT
        //        ),
          //      0
            //);
        
            goal = setpoint;

            voltage = joystickOverride.getAsDouble() * 9.0;
        }
        Logger.recordOutput("Elevator/passedInVoltage", voltage);
        setMotorSpeed(voltage);
        Logger.recordOutput("Elevator/Goal", goal.position);
        Logger.recordOutput("Elevator/Setpoint", setpoint.position);
        Logger.recordOutput("Elevator/Homed", isZeroed);
    }
    // Look into soft limits: https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/softlimitconfig
    public void setMotorSpeed(double voltage) {
        if (voltage < 0) {
            if (io.getBottomLimitSwitch()) {
                io.runVoltage(0);
            }
            else {
                io.runVoltage(voltage);
            }
        } else {
            if(inputs.positionMeters > ElevatorConstants.MAX_HEIGHT) {
                io.runVoltage(0);
            } else {
                io.runVoltage(voltage);
            }
        }
    }

    public void setJoystickSupplier(DoubleSupplier joystick) {
        this.joystickOverride = joystick;
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
    
    public boolean atHeight() {
        return MathUtil.isNear(goal.position, inputs.positionMeters, 0.05); // 2cm tolerance
    }

    public void init() {
        voltage = 0;
    }
}   
