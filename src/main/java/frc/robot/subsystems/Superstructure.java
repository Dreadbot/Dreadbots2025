package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure {

    private final Wrist wrist;
    private final Elevator elevator;
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
            wrist.setAngleDegrees(level.angle)
                .andThen(Commands.waitUntil(wrist::atSetpoint))
                .andThen(elevator.riseTo(level.height)),

            elevator.riseTo(level.height)
                .andThen(wrist.setAngleDegrees(level.angle)),

            () -> wrist.isInDangerZone() && (level == SuperstructureState.L4)
        );
    }
    //Height, Angle (degrees)
    public static enum SuperstructureState {
        L1(0.634, 110.0),
        L2(0.996, 65.6),
        L3(1.415, 65.6),
        L4(2.120, 51.0),
        STOW(0.634, 8.0),
        PICKUP(0.84, 145.0);
        SuperstructureState(double height, double angle) {
            this.height = height;
            this.angle = angle;
        }
        public final double height;
        public final double angle;
    }
}
