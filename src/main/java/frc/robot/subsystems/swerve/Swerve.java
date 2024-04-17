package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.base.SwerveBaseIO;
import frc.robot.subsystems.swerve.base.SwerveBaseSim;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class Swerve extends SubsystemBase {
    private SwerveBaseIO base;
    public Swerve() {
        base = RobotBase.isReal() ? null : new SwerveBaseSim();
    }

    @Override
    public void periodic() {
        base.periodic();
    }

    public Command angleCentric(CommandXboxController xbox) {
        return run(() -> {
            double coefficient = Math.max(1-xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
        });
    }
}
