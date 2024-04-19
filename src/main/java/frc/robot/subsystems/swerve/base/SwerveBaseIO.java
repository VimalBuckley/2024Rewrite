package frc.robot.subsystems.swerve.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public abstract class SwerveBaseIO {
    public abstract void setSpeeds(ChassisSpeeds speeds);
    public abstract void setPose(Pose2d pose);
    public abstract void periodic();
    public abstract ChassisSpeeds getSpeeds();
    public abstract Pose2d getPose();
    public abstract SwerveDriveKinematics getKinematics();

    public static record SwerveModuleConfig(Translation2d translation, double wheelDiameter, double driveRatio, double angleRatio) {}
}
