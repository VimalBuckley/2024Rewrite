package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.base.SwerveBaseIO.SwerveModuleConfig;

public class SwerveConstants {
    public static final double MAX_FORWARD_SPEED = 6;
    public static final double MAX_SIDEWAYS_SPEED = 6;
    public static final double MAX_ROTATIONAL_SPEED = 4;
    public static final double MIN_COEFFICIENT = 0.2;

    public static final SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(0.2974, 0.2974), 0.1016, 1/7.5, 1/6.75
    );
    public static final SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(0.2974, -0.2974), 0.1016, 1/7.5, 1/6.75
    );
    public static final SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(-0.2974, 0.2974), 0.1016, 1/7.5, 1/6.75
    );
    public static final SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(-0.2974, -0.2974), 0.1016, 1/7.5, 1/6.75
    );
}
