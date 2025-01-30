package frc.robot.util;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoAlignUtil {

	private static double fieldSizeX = Units.feetToMeters(57.573);
	private static double fieldSizeY = Units.feetToMeters(26.417);
	public static List<Pose2d> POIs;
	private static Alert createdPOIAlert = new Alert("Generated POI List!", AlertType.kInfo);
	/**
	 * This function generates all the POIs for the Autoalign command, requires that we know which side of the field we are on, so needs to called at runtime
	 */
	public static void buildPOIList() {
		Pose2d[] tmp = new Pose2d[6];
		// Reef Auto Align Poses
		Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
		Translation2d firstSide = new Translation2d(3.229, 4.026);
		for(int i = 0; i < 6; i++) {
			Translation2d side = firstSide.rotateAround(reefCenter, Rotation2d.fromDegrees(60 * (i - 1)));
			tmp[i] = getAlliancePOI(
				new Pose2d(side, Rotation2d.fromDegrees(60 * (i - 1)))
			);
		}

		POIs = List.of(tmp);
	}

	/**
	 * Takes a POI and flips it to the other side of the field if alliance is red
	 * @param poi The POI to flip
	 * @return The Alliance specific POI
	 */
	public static Pose2d getAlliancePOI(Pose2d poi) {

		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			Translation2d rotatedPosition = new Translation2d(fieldSizeX - poi.getX(), fieldSizeY - poi.getY());
			Rotation2d flippedRotation = poi.getRotation().plus(Rotation2d.kPi);
			return new Pose2d(rotatedPosition, flippedRotation);
		}

		return poi;
	}

	public static Command createPOIListCommand() {
		return Commands.runOnce(() -> {
			AutoAlignUtil.buildPOIList();
			Logger.recordOutput("AutoAlign/POIListGenerated", true);
			createdPOIAlert.set(true);
		});
	}
}
