package frc.robot.subsystems.wrist;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private WristIO io;
    private final PIDController pid = new PIDController(1.0, 0.0, 0);
    private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0, 0.0);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(180, 180));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private double goalAngle;
    private State desiredWristState;
    public double joystickOverride;


    public Wrist(WristIO io) {
        this.io = io;
        desiredWristState = new State(0, 0);
        this.joystickOverride = 0.0;
    } 

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        goal = new TrapezoidProfile.State(goalAngle, 0);
        Logger.recordOutput("Wrist/SetpointPosition", setpoint.position);
        Logger.recordOutput("Wrist/GoalAngle", goalAngle);
        Logger.recordOutput("Wrist/GoalPosition", goal.position);
        Logger.recordOutput("Wrist/AtSetpoint", atSetpoint());
        Logger.recordOutput("Wrist/InDangerZone", isInDangerZone());
        setpoint = profile.calculate(0.02, setpoint, goal);
        io.runVoltage(pid.calculate(inputs.rotationDegrees, setpoint.position) + feedforward.calculate(Units.degreesToRadians(setpoint.position) ,setpoint.velocity));
        
        
        if (Math.abs(joystickOverride) > 0.08) {
            
            this.desiredWristState = new State(
                MathUtil.clamp(
                    this.desiredWristState.position + joystickOverride * WristConstants.WRIST_JOYSTICK_SLEW_VALUE,
                    0.000,
                    WristConstants.WRIST_MAX_ANGLE
                ),
                0
            );
        }
    }

    public Command setAngleDegrees(double angle) {
         return runOnce(
            () -> {
                goalAngle = angle;
             } );
    }

    public Command setJoystickOverride(DoubleSupplier joystickValue) {
        return runOnce (
            () -> {
                joystickOverride = joystickValue.getAsDouble();
                System.out.println("Joystick Override: " + joystickValue.getAsDouble());
            }
        );
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
        return MathUtil.isNear(goalAngle, inputs.rotationDegrees, 0.4); // 0.2 degrees
    }

    /**
     * Gets if the Wrist is in danger zone, see START_DANGER_ZONE for more information.
     * @return True of false whether we are in danger zone or not.
     */
    public boolean isInDangerZone() {
        return getAngle() > WristConstants.START_DANGER_ZONE;
    }

}
