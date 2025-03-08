package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffector;

public class AutoCommands {
    private AutoFactory factory;
    private Drive drive;
    private Superstructure superstructure;
    private EndEffector endEffector;

    public AutoCommands(Drive drive, Superstructure superstructure, EndEffector endEffector) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.endEffector = endEffector;

        this.endEffector = endEffector;
        this.factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            drive::logTrajectory
        );
    }

    public Command midBargeC2Low() {
        return Commands.sequence(
            factory.resetOdometry("MidBarge-C2"),
            factory.trajectoryCmd("MidBarge-C2")
                .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L1))
                .andThen(drive.stopDrive()),
            endEffector.outtake().withTimeout(2.0)
        );
    }
    public Command midBargeC2High() {
        return Commands.sequence(
            factory.resetOdometry("MidBarge-C2", 0),
            factory.trajectoryCmd("MidBarge-C2", 0)
                .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L3))
                .andThen(drive.stopDrive()),
            superstructure.requestSuperstructureState(SuperstructureState.L4),
            factory.trajectoryCmd("MidBarge-C2", 1)
                .andThen(drive.stopDrive()),
            endEffector.outtake().withTimeout(2.0)
        );
    }

    public Command midD2High() {
        return Commands.sequence(
            factory.resetOdometry("Middle-D2", 0),
            factory.trajectoryCmd("Middle-D2", 0)
                .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L4))
                .andThen(drive.stopDrive()),
            factory.trajectoryCmd("Middle-D2", 1)
                .andThen(drive.stopDrive()),
            endEffector.outtake().withTimeout(1.0),
            factory.trajectoryCmd("Middle-D2", 2)
                .alongWith(Commands.waitSeconds(0.6).andThen(superstructure.requestSuperstructureState(SuperstructureState.STOW)))
                .andThen(drive.stopDrive())
        );
    }

    public Command midProcessorE1High() {
        return Commands.sequence(
            factory.resetOdometry("MidProcessor-E1", 0),
            factory.trajectoryCmd("MidProcessor-E1", 0)
                .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L3))
                .andThen(drive.stopDrive()),
            superstructure.requestSuperstructureState(SuperstructureState.L4),
            factory.trajectoryCmd("MidProcessor-E1", 1)
                .andThen(drive.stopDrive()),
            endEffector.outtake().withTimeout(1.0),
            factory.trajectoryCmd("MidProcessor-E1", 2)
                .alongWith(Commands.waitSeconds(2.5).andThen(superstructure.requestSuperstructureState(SuperstructureState.STOW)))
                .andThen(drive.stopDrive())
        );
    }
    public Command midProcessorE1PickupHigh() {
        return Commands.sequence(
            factory.resetOdometry("MidProcessor-E1-FarPickup", 0),
            factory.trajectoryCmd("MidProcessor-E1-FarPickup", 0)
                .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L3))
                .andThen(drive.stopDrive()),
            superstructure.requestSuperstructureState(SuperstructureState.L4),
            factory.trajectoryCmd("MidProcessor-E1-FarPickup", 1)
                .andThen(drive.stopDrive()),
            endEffector.outtake().withTimeout(1.0),
            factory.trajectoryCmd("MidProcessor-E1-FarPickup", 2)
                .alongWith(Commands.waitSeconds(1.0).andThen(superstructure.requestSuperstructureState(SuperstructureState.PICKUP)))
                .andThen(drive.stopDrive())
        );
    }

    // public Command midProcessorE1F1High() {
    //     return Commands.sequence(
    //         factory.resetOdometry("MidProcessor-E1F1", 0),
    //         factory.trajectoryCmd("MidProcessor-E1F1", 0)
    //             .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L3))
    //             .andThen(drive.stopDrive()),
    //         superstructure.requestSuperstructureState(SuperstructureState.L4),
    //         factory.trajectoryCmd("MidProcessor-E1F1", 1)
    //             .andThen(drive.stopDrive()),
    //         endEffector.outtake().withTimeout(1.0),
    //         factory.trajectoryCmd("MidProcessor-E1F1", 2)
    //             .alongWith(Commands.waitSeconds(0.1)
    //             .andThen(superstructure.requestSuperstructureState(SuperstructureState.PICKUP)))
    //             .andThen(drive.stopDrive()),
    //         endEffector.intake().withTimeout(1.0),
    //         factory.trajectoryCmd("MidProcessor-E1F1", 3)
    //             .andThen(superstructure.requestSuperstructureState(SuperstructureState.L4))
    //             .andThen(drive.stopDrive()),
    //         factory.trajectoryCmd("MidProcessor-E1F1", 4)
    //             .andThen(drive.stopDrive()),
    //         endEffector.outtake().withTimeout(1.0)
    //     );
    // }

    public Command midBargeC2B1High() {
        return Commands.sequence(
            factory.resetOdometry("MidBarge-C2B1", 0),
            factory.trajectoryCmd("MidBarge-C2B1", 0)
                .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L3))
                .andThen(drive.stopDrive()),
            superstructure.requestSuperstructureState(SuperstructureState.L4),
            factory.trajectoryCmd("MidBarge-C2B1", 1)
                .andThen(drive.stopDrive()),
            endEffector.outtake().withTimeout(0.5),
            factory.trajectoryCmd("MidBarge-C2B1", 2)
                .alongWith(Commands.waitSeconds(0.1)
                .andThen(superstructure.requestSuperstructureState(SuperstructureState.PICKUP)))
                .andThen(drive.stopDrive()),
            endEffector.intake().until(endEffector::hasCoral),
            factory.trajectoryCmd("MidBarge-C2B1", 3)
                .andThen(superstructure.requestSuperstructureState(SuperstructureState.L3))
                .andThen(drive.stopDrive()),
            factory.trajectoryCmd("MidBarge-C2B1", 4)
                .alongWith(superstructure.requestSuperstructureState(SuperstructureState.L4))
                .andThen(drive.stopDrive()),
            endEffector.outtake().withTimeout(0.5)
        );
    }

    public Command midProcessorE2F2FarPickup() {
        return Commands.sequence(
            factory.resetOdometry("MidProcessor-E2F2-FarPickup"), 
            factory.trajectoryCmd("MidProcessor-E2F2-FarPickup", 0), // run first segment of path (from starting location to first scoring location)
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)), // run Scoring sequence (elevator and wrist to l4, outtake, back down to pickup)
            endEffector.outtake().withTimeout(0.5),
            superstructure.requestSuperstructureState(SuperstructureState.PICKUP).andThen(Commands.waitUntil(superstructure::isFinished)),
            factory.trajectoryCmd("MidProcessor-E2F2-FarPickup", 1), // run second segment of path (from first scoring location to pickup location)
            endEffector.intake().until(endEffector::hasCoral), // run intake until we have game piece
            factory.trajectoryCmd("MidProcessor-E2F2-FarPickup", 2), // run third segment of path (from pickup location to second scoring location)
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)), // run Scoring sequence (elevator and wrist to l4, outtake, back down to pickup)
            endEffector.outtake().withTimeout(0.5),
            superstructure.requestSuperstructureState(SuperstructureState.STOW)
        );
    }

    public Command MidBargeC2B2ClosePickup(){
        return Commands.sequence(
            factory.resetOdometry("MidBarge-C2B2-ClosePickup",0),
            factory.trajectoryCmd("MidBarge-C2B2-ClosePickup",0),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.PICKUP).andThen(Commands.waitUntil(superstructure::isFinished)),
            factory.trajectoryCmd("MidBarge-C2B2-ClosePickup",1),
            endEffector.intake().until(endEffector::hasCoral),
            factory.trajectoryCmd("MidBarge-C2B2-ClosePickup",2),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.STOW)
        );
    }

    public Command MiddleD2(){
        return Commands.sequence(
            factory.resetOdometry("Middle-D2",0),
            factory.trajectoryCmd("Middle-D2",0),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.STOW)
        );
    }

    public Command MidProcessorE2(){
        return Commands.sequence(
            factory.resetOdometry("MidProcessor-E2",0),
            factory.trajectoryCmd("MidProcessor-E2",0),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.STOW)
        );
    }

    public AutoRoutine midProcessorE1F1High(){
        AutoRoutine routine = factory.newRoutine("MidProcessor-E1F1-High");
        AutoTrajectory midProcessorToSlow = routine.trajectory("MideProcessor-E1F1", 0);
        AutoTrajectory slowToE1 = routine.trajectory("MideProcessor-E1F1", 1);
        AutoTrajectory e1ToPickup = routine.trajectory("MideProcessor-E1F1", 2);
        AutoTrajectory pickupToSlow = routine.trajectory("MideProcessor-E1F1", 3);
        AutoTrajectory slowToF1 = routine.trajectory("MideProcessor-E1F1", 4);

        routine.active().onTrue(
            Commands.sequence(
                midProcessorToSlow.resetOdometry(),
                midProcessorToSlow.cmd(),
                superstructure.requestSuperstructureState(SuperstructureState.L3)
            )
        );

        midProcessorToSlow.done().onTrue(
            Commands.sequence(
                superstructure.requestSuperstructureState(SuperstructureState.L4),
                slowToE1.cmd()
            )
        );

        slowToE1.done().onTrue(endEffector.outtake());
        slowToE1.done().and(() -> !endEffector.hasCoral()).onTrue(
            Commands.sequence(
                e1ToPickup.cmd(),
                superstructure.requestSuperstructureState(SuperstructureState.PICKUP)
            )
        );

        slowToE1.done().onTrue(endEffector.outtake());


        e1ToPickup.done().and(endEffector::hasCoral).onTrue(Commands.sequence(
            pickupToSlow.cmd(),
            superstructure.requestSuperstructureState(SuperstructureState.L3)
        ));

        pickupToSlow.done().onTrue(Commands.sequence(
            slowToF1.cmd(),
            superstructure.requestSuperstructureState(SuperstructureState.L4)
        ));

        return routine;
    }

    public Command midBargeC2B1B2(){
        // AutoRoutine routine = factory.newRoutine("MidBargeC2B1B2h");
        // AutoTrajectory midProcessorToSlow = routine.trajectory("MidBarge-C2B1B2", 0);
        // AutoTrajectory slowToE1 = routine.trajectory("MidBarge-C2B1B2", 1);
        // AutoTrajectory e1ToPickup = routine.trajectory("MidBarge-C2B1B2", 2);
        // AutoTrajectory pickupToSlow = routine.trajectory("MidBarge-C2B1B2", 3);
        // AutoTrajectory slowToF1 = routine.trajectory("MidBarge-C2B1B2", 4);

        //basic code that will be updated
        return Commands.sequence(
            factory.resetOdometry("MidBarge-C2B1B2",0),
            factory.trajectoryCmd("MidBarge-C2B1B2",0),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.PICKUP).andThen(Commands.waitUntil(superstructure::isFinished)),
            factory.trajectoryCmd("MidBarge-C2B1B2",1),
            endEffector.intake().until(endEffector::hasCoral),
            factory.trajectoryCmd("MidBarge-C2B1B2",2),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),  
            superstructure.requestSuperstructureState(SuperstructureState.PICKUP).andThen(Commands.waitUntil(superstructure::isFinished)),
            factory.trajectoryCmd("MidBarge-C2B1B2",3),
            endEffector.intake().until(endEffector::hasCoral),
            superstructure.requestSuperstructureState(SuperstructureState.L4).andThen(Commands.waitUntil(superstructure::isFinished)),
            endEffector.outtake().withTimeout(.5),
            superstructure.requestSuperstructureState(SuperstructureState.STOW)
        );
    }

    public Command WheelRadiCalibration() {
        return Commands.sequence(
            factory.resetOdometry("WheelRadiCalibration"),
            factory.trajectoryCmd("WheelRadiCalibration")
        );
    }
}
