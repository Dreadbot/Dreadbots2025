package frc.robot.subsystems.wrist;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private WristIO io;
    private final PIDController pid = new PIDController(0.1, 0.0, 0);
    private final ArmFeedforward feedforward = new ArmFeedforward(0.26, 0.15, 0.03);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(540, 840));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    public DoubleSupplier joystickOverride;
    public double voltage;


    public Wrist(WristIO io) {
        this.io = io;
        this.joystickOverride = () -> 0.0;
        this.voltage = 0;
        io.updateInputs(inputs);
        goal = new TrapezoidProfile.State(inputs.rotationDegrees, 0);
        setpoint = goal;
    } 

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(inputs.rotationDegrees, 0);
            goal = setpoint;
        }

        Logger.recordOutput("Wrist/SetpointPosition", setpoint.position);
        Logger.recordOutput("Wrist/GoalPosition", goal.position);
        Logger.recordOutput("Wrist/AtSetpoint", atSetpoint());
        Logger.recordOutput("Wrist/InDangerZone", isInDangerZone());
        setpoint = profile.calculate(0.02, setpoint, goal);
        voltage = pid.calculate(inputs.rotationDegrees, setpoint.position) 
        + feedforward.calculate(Units.degreesToRadians(setpoint.position - 90) ,setpoint.velocity); //convert so 0 degrees is horizontal
        
        if (Math.abs(joystickOverride.getAsDouble()) > 0.08) {
            
            // this.desiredWristState = new State(
            //     MathUtil.clamp(
            //         this.desiredWristState.position + joystickOverride * WristConstants.WRIST_JOYSTICK_SLEW_VALUE,
            //         0.000,
            //         WristConstants.WRIST_MAX_ANGLE
            //     ),
            //     0
            // );
            setpoint = new State(inputs.rotationDegrees, 0);
            goal = setpoint;
            voltage = joystickOverride.getAsDouble() * WristConstants.WRIST_JOYSTICK_SLEW_VALUE;
        }
        io.runVoltage(voltage);
    }

    public Command setAngleDegrees(double angle) {
        return runOnce(
            () -> {
                goal = new TrapezoidProfile.State(angle, 0);
             } );
    }

    public void setJoystickOverride(DoubleSupplier joystickSupplier) {
        this.joystickOverride = joystickSupplier;
    }

    public Command setAtZero() {
        return runOnce(
            () -> {
            } );
    }
    
    public double getAngle() {
        return inputs.rotationDegrees;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(goal.position, inputs.rotationDegrees, 4.0); // 4.0 degrees
    }

    /**
     * Gets if the Wrist is in danger zone, see START_SAFE_ZONE for more information.
     * @return True of false whether we are in danger zone or not.
     */
    public boolean isInDangerZone() {
        return getAngle() < WristConstants.START_SAFE_ZONE || getAngle() > WristConstants.END_SAFE_ZONE;
    }

}
