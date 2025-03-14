// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.slapdownAlgae.SlapdownAlgae;
import frc.robot.subsystems.slapdownAlgae.SlapdownAlgaeIO;
import frc.robot.subsystems.slapdownAlgae.SlapdownAlgaeIOSim;
import frc.robot.subsystems.slapdownAlgae.SlapdownAlgaeIOSparkMax;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSolenoid;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkFlex;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.endEffector.EndEffectorIOSparkFlex;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIONetworkTables;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSparkMax;
import frc.robot.util.visualization.VisualizationManager;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Climb climb;
  private final SlapdownAlgae slapdownAlgae;
  private final VisualizationManager vizManager;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);
  private final Alert autoInitFaliure = new Alert("Failed to load Auto Paths!", AlertType.kError);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:

        drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));
      endEffector = new EndEffector(new EndEffectorIOSparkFlex());
      wrist = new Wrist(new WristIOSparkMax());
      vision = new Vision(drive::addVisionMeasurement, new VisionIONetworkTables());
      slapdownAlgae = new SlapdownAlgae(new SlapdownAlgaeIOSparkMax());
      elevator = new Elevator(new ElevatorIOSparkFlex());
      climb = new Climb(new ClimbIOSolenoid());
      break;
        

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        wrist = new Wrist(new WristIOSim());
        vision = new Vision(drive::addVisionMeasurement, new VisionIONetworkTables());
        slapdownAlgae = new SlapdownAlgae(new SlapdownAlgaeIOSim());
        climb = new Climb(new ClimbIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        slapdownAlgae = new SlapdownAlgae(new SlapdownAlgaeIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }
    vizManager = new VisualizationManager(elevator::getHeight, wrist::getAngle, slapdownAlgae::getAngle);
    superstructure = new Superstructure(elevator, wrist);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    try {
        autoChooser.addDefaultOption("Choreo Test Auto", AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Processor-A2-FarPickup")));
    } catch(Exception e) {
        autoInitFaliure.setText("Failed to Init Choreo Test Auto!");
        autoInitFaliure.set(true);
    }
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -primaryController.getLeftY(),
            () -> -primaryController.getLeftX(),
            () -> -primaryController.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    /* 
     * Keybinds for the secondary controller
     * Focuses on the coral pieces
     * Elevator / Wrist / Endeffector
     */

    //Home
    secondaryController.a().onTrue(superstructure.requestSuperstructureState(SuperstructureState.STOW));

    //L1 - L4
    secondaryController.povUp().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L4));
    secondaryController.povLeft().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L3));
    secondaryController.povRight().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L2));
    secondaryController.povDown().onTrue(superstructure.requestSuperstructureState(SuperstructureState.L1));

    //intake sequence
    secondaryController.leftTrigger().onTrue(superstructure.requestSuperstructureState(SuperstructureState.PICKUP)
        .alongWith(endEffector.intake()));
    
    //intake / outtake
    secondaryController.leftBumper().onTrue(endEffector.intake());
    secondaryController.rightBumper().onTrue(endEffector.outtake());

    //Reset elevator / wrist
    secondaryController.start().onTrue(elevator.requestZero());
    secondaryController.back().onTrue(wrist.setAtZero());

    // secondaryController.rightTrigger().onTrue(elevator.riseTo(Units.inchesToMeters(60)));

    //manual override buttons
    elevator.setDefaultCommand(elevator.setJoystickOverride(() -> -secondaryController.getLeftY()));
    wrist.setDefaultCommand(wrist.setJoystickOverride(() -> -secondaryController.getRightY()));
    
    //climb sequence
    primaryController.y().whileTrue(climb.climbSequence());

    //Slapdown Algae Buttons (Left Trigger Intakes wheels/ Right Trigger Outakes wheels) (D-pad Up will pull in the intake system while D-pad down will push the intake system out to grab Algae) 
    primaryController.x().whileTrue(slapdownAlgae.setAngleDegrees(-80).andThen(slapdownAlgae.intake()));
    primaryController.b().whileTrue(slapdownAlgae.setAngleDegrees(80).andThen(slapdownAlgae.outtake()));
    // controller.povUp().toggleOnTrue(slapdownAlgae.setAngleDegrees(90));
    // controller.povDown().toggleOnTrue(slapdownAlgae.setAngleDegrees(0));  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void autonomousInit(){
    elevator.init();
  }

  public void teleopInit(){
    elevator.init();
  }
}
