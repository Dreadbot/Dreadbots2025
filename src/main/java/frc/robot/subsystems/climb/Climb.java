package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    
    private ClimbIO io;
    private Climb climb;
    private boolean extended = false;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io) { 
        this.io = io;
    }

    @Override
    public void periodic() {}
    
    public void isExtended(boolean extended) {
        this.extended = true;
    }

    public void isRetracted(boolean extended) {
        this.extended = false;
    }

    public boolean getExtendedClimb() {
        return inputs.extendedClimb;
    }

    public boolean getExtendedLock() {
        return inputs.extendedLock;
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

   public Command climbSequence() {
    return swapStatusLock()
    .andThen(Commands.waitSeconds(1)
    .andThen(swapStatusClimb()));
   }
    
}
