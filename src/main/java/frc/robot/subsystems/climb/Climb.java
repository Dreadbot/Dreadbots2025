package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

public class Climb extends SubsystemBase {
    
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    
    public Climb(ClimbIO io) { 
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public Command swapStatusClimb() {
         return runOnce(() -> {
            io.setClimbEnabled(!inputs.extendedClimb);
         });
    }

    public Command swapStatusLock() {
        return runOnce(() -> {
            io.setLockEnabled(!inputs.extendedLock);
        });
   }
   public Command swapStatusClaw() {
    return runOnce(() -> {
        io.setClawEnabled(!inputs.extendedClaw);
    });
    }

   public Command climbSequence() {
    return swapStatusClaw()
        .andThen(Commands.waitSeconds(1.0))
        .andThen(swapStatusLock())
        .andThen(Commands.waitSeconds(1.0))
        .andThen(swapStatusClimb());
   }
    
}
