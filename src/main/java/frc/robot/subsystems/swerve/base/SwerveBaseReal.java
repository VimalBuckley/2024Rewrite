package frc.robot.subsystems.swerve.base;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.hardware.Limelight2;
import frc.robot.hardware.NavX;
import frc.robot.hardware.Limelight2.PoseEstimate;
import frc.robot.utilities.ExtendedMath;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class SwerveBaseReal extends SwerveBaseIO {
    private NavX gyro;
    private Limelight2[] limelights;
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator estimator;
    public SwerveBaseReal() {
        gyro = new NavX(Port.kMXP);
        limelights = new Limelight2[] {
            new Limelight2("limelight-hehehe", 0)
        };
        modules = new SwerveModule[] {
            new SwerveModule(
				FRONT_LEFT_DRIVE_MOTOR,
				FRONT_LEFT_ANGLE_MOTOR,
				FRONT_LEFT_MODULE_CONFIG
			),
			new SwerveModule(
				FRONT_RIGHT_DRIVE_MOTOR,
				FRONT_RIGHT_ANGLE_MOTOR,
				FRONT_RIGHT_MODULE_CONFIG
			),
			new SwerveModule(
				BACK_LEFT_DRIVE_MOTOR,
				BACK_LEFT_ANGLE_MOTOR,
				BACK_LEFT_MODULE_CONFIG
			),
			new SwerveModule(
				BACK_RIGHT_DRIVE_MOTOR,
				BACK_RIGHT_ANGLE_MOTOR,
				BACK_RIGHT_MODULE_CONFIG
			)
        };
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        estimator = new SwerveDrivePoseEstimator(kinematics, gyro.getOffsetedAngle(), getModulePositions(), new Pose2d());
    }

    @Override
    public void setSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			ChassisSpeeds.discretize(speeds, 0.02)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			MAX_MODULE_SPEED
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
    }

    @Override
    public void setPose(Pose2d pose) {
        estimator.resetPosition(gyro.getUnwrappedAngle(), getModulePositions(), pose);
        setAngle(pose.getRotation());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        gyro.zeroWithOffset(angle);
    }

    @Override
    public void periodic() {
        estimator.update(gyro.getUnwrappedAngle(), getModulePositions());
        boolean speedLimit =
            (ExtendedMath.within(getSpeeds(), new ChassisSpeeds(), new ChassisSpeeds(1, 1, 2 * Math.PI)) || 
			!DriverStation.isAutonomous());
        for (Limelight2 camera : limelights) {
            PoseEstimate estimate = camera.getPoseMT2(
                gyro.getOffsetedAngle(), new Rotation2d(getSpeeds().omegaRadiansPerSecond));
            if (estimate.exists() && speedLimit && (estimate.tagCount() > 1 || estimate.averageDistance() < 4))
                estimator.addVisionMeasurement(estimate.pose(), Timer.getFPGATimestamp() - estimate.latencySeconds());
        }
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    private Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}

	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getPosition();
		}
		return positions;
	}
}