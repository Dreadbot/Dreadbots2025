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
        L1(0.634, 135.0),
        L2(Units.inchesToMeters(39.0), 55.0),
        L3(Units.inchesToMeters(55.0), 55.0),
        L4(Units.inchesToMeters(81.0), 50.0),
        STOW(0.634, 8.0),
        PICKUP(0.8, 145.0);
        SuperstructureState(double height, double angle) {
            this.height = height;
            this.angle = angle;
        }
        public final double height;
        public final double angle;
    }
}
