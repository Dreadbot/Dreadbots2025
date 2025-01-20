package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private WristIO io;
    public PIDController pid = new PIDController(1.5, 0.005, 0);
    public ArmFeedforward feedforward = new ArmFeedforward(0.0, 1, 0.25);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(180, 180));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private double goalAngle;


    public Wrist(WristIO io) {
        this.io = io;
    } 

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        goal = new TrapezoidProfile.State(goalAngle, 0);
        Logger.recordOutput("SetpointPosition", setpoint.position);
        Logger.recordOutput("GoalAngle", goalAngle);
        Logger.recordOutput("GoalPosition", goal.position);
        setpoint = profile.calculate(0.02, setpoint, goal);
        io.runVoltage(pid.calculate(inputs.rotationDegrees, setpoint.position) + feedforward.calculate(Units.degreesToRadians(setpoint.position) ,setpoint.velocity));
    }

    public Command setAngleDegrees(double angle) {
        
        return run(
            () -> {
                goalAngle = angle;
             } );
    }

}
