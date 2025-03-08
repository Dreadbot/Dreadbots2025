package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

public class Climb extends SubsystemBase {
    
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    public boolean isClimbed = false; 
    
    public Climb(ClimbIO io) { 
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public Command extendClimb() {
         return runOnce(() -> {
            io.setClimbState(DoubleSolenoid.Value.kForward);
         });
    }
    public Command retractClimb() {
        return runOnce(() -> {
           io.setClimbState(DoubleSolenoid.Value.kReverse);
        });
   }

   public Command extendLock() {
        return runOnce(() -> {
            io.setLockState(DoubleSolenoid.Value.kForward);
        });
    }

   public Command retractLock() {
        return runOnce(() -> {
            io.setLockState(DoubleSolenoid.Value.kReverse);
        });
   }

   public Command extendClaw() {
    return runOnce(() -> {
        io.setClawEnabled(true);
    });
   }

   public Command retractClaw() {
    return runOnce(() -> {
        io.setClawEnabled(false);
    });
   }
   public Command climbSequence() {
        return extendClaw()
            .andThen(Commands.waitSeconds(0.5))
            .andThen(extendLock())
            .andThen(Commands.waitSeconds(0.5))
            .andThen(extendClimb()
            .andThen(() -> {isClimbed = true;}));
   }

   public Command init() {
    return retractLock()
        .andThen(retractClimb())
        .andThen(Commands.waitSeconds(0.5))
        .andThen(retractClaw())
        .andThen(() -> {isClimbed = false;});
   }

   public boolean getIsClimbed(){
    return isClimbed;
   }

   public Command climb(){
    return Commands.either(init(), climbSequence(), () -> getIsClimbed()); // Declimbs if climbed, climbs if not climbed
   }
}
