package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure {

    private final Wrist wrist;
    private final Elevator elevator;
    private SuperstructureState previousState = SuperstructureState.STOW;
    public Superstructure(Elevator elevator, Wrist wrist) {
        this.wrist = wrist;
        this.elevator = elevator;
    }

    public Command requestSuperstructureState(SuperstructureState level) {
        //If we are in dangerZone, run angle first, and then go to position
        /*
         * Commands.either(
         * OnTrue,
         * OnFalse,
         * Conditional
         * )
         */
        return Commands.either(
            wrist.setAngleDegrees(SuperstructureState.STOW.angle)
                .andThen(elevator.riseTo(level.height))
                .andThen(Commands.waitUntil(elevator::inSafeZone))
                .andThen(wrist.setAngleDegrees(level.angle)),
            elevator.riseTo(level.height)
                .andThen(wrist.setAngleDegrees(level.angle)),

            () -> (previousState == SuperstructureState.L4 && level == SuperstructureState.PICKUP) // Pickup requires a wrist angle that is inside danger zone
        ).until(() -> wrist.atSetpoint() && elevator.atHeight()).finallyDo(() -> previousState = level);
    }

    public boolean isFinished(){
        return wrist.atSetpoint() && elevator.atHeight();
    }
    //Height, Angle (degrees)
    public static enum SuperstructureState {
        L1(0.634, 110.0),
        L2(0.996, 65.6),
        L3(1.415, 65.6),
        L4(2.137, 49.0),
        STOW(0.627, 8.0),
        PICKUP(0.87, 135.0),
        KNOCKOUT_L2(0.97, 90.0),
        KNOCKOUT_L3(1.38, 90.0);
        SuperstructureState(double height, double angle) {
            this.height = height;
            this.angle = angle;
        }
        public final double height;
        public final double angle;
    }
}
