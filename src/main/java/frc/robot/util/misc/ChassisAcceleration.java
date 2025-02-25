package frc.robot.util.misc;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisAcceleration extends ChassisSpeeds {

    /**
     * Relies on Chassis Speeds, class for clarity only
     * @param aXMetersPerSecSquared Forwards Acceleration.
     * @param aYMetersPerSecSquared Sideways Acceleration.
     * @param alphaRadiansPerSecSquared Rotational Acceleration.
     */
    public ChassisAcceleration(double aXMetersPerSecSquared, double aYMetersPerSecSquared, double alphaRadiansPerSecSquared) {
        super(aXMetersPerSecSquared, aYMetersPerSecSquared, alphaRadiansPerSecSquared);
    }

    /**
     * Gets Chassis Acceleration in form that we can log
     * @return Chassis Speed as Chassis Acceleration
     */
    public ChassisSpeeds toLoggableOutput() {
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
    
}
