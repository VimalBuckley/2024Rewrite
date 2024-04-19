package frc.robot.subsystems.swerve.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
public class SwerveBaseSim extends SwerveBaseIO {
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;
    private SwerveDriveKinematics kinematics;
    public SwerveBaseSim() {
        currentPose = new Pose2d();
        currentSpeeds = new ChassisSpeeds();
        kinematics = new SwerveDriveKinematics(
            FRONT_LEFT_MODULE_CONFIG.translation(),
            FRONT_RIGHT_MODULE_CONFIG.translation(),
            BACK_LEFT_MODULE_CONFIG.translation(),
            BACK_RIGHT_MODULE_CONFIG.translation()
        ); 
    }
    @Override
    public void setSpeeds(ChassisSpeeds speeds) {
        currentSpeeds = speeds;
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return currentSpeeds;
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        var fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
        currentPose = new Pose2d(
            currentPose.getX() + fieldRelative.vxMetersPerSecond * 0.02,
            currentPose.getY() + fieldRelative.vyMetersPerSecond * 0.02,
            currentPose.getRotation().plus(Rotation2d.fromRotations(fieldRelative.omegaRadiansPerSecond * 0.02))
        );
    }
    
}
