package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.base.SwerveBaseIO;
import frc.robot.subsystems.swerve.base.SwerveBaseReal;
import frc.robot.subsystems.swerve.base.SwerveBaseSim;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase implements LoggableInputs {
    private SwerveBaseIO base;
    private Rotation2d targetAngle;
    private PIDController anglePID;
    public Swerve() {
        base = RobotBase.isReal() ? new SwerveBaseReal() : new SwerveBaseSim();
        targetAngle = new Rotation2d();
        anglePID = new PIDController(5, 0, 0);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
        AutoBuilder.configureHolonomic(
            base::getPose, 
            base::setPose, 
            base::getSpeeds, 
            base::setSpeeds, 
            new HolonomicPathFollowerConfig(5, 0.39878808909, new ReplanningConfig(true, true)), 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
            this
        );
    }

    @Override
    public void periodic() {
        base.periodic();
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Target Angle", targetAngle.getDegrees());
        Logger.recordOutput("Pose", base.getPose());
    }

    @Override
    public void fromLog(LogTable table) {}

    public Command angleCentric(CommandXboxController xbox) {
        return run(() -> {
            double coefficient = Math.max(1-xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
            double allianceCoefficient = 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                1 : -1;
            double forward = coefficient * -xbox.getLeftY() * MAX_FORWARD_SPEED;
            double sideways = coefficient * -xbox.getLeftX() * MAX_SIDEWAYS_SPEED;
            forward *= allianceCoefficient;
            sideways *= allianceCoefficient;
            double angleCoefficient = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1;
                if (Math.abs(xbox.getRightY()) > 0.5)
                    targetAngle = Rotation2d.fromDegrees(90 + angleCoefficient * 90 * Math.signum(-xbox.getRightY()));
            else if (xbox.getHID().getRightStickButton()) 
                targetAngle = Rotation2d.fromDegrees(-90);
            else if (xbox.getHID().getLeftBumper())
                targetAngle = Rotation2d.fromDegrees(90);
            else if (xbox.getHID().getYButton()) {
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                    targetAngle = Rotation2d.fromDegrees(-60);
                else
                    targetAngle = Rotation2d.fromDegrees(-120);
            }
            else if (xbox.getHID().getXButton()) {
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                    targetAngle = Rotation2d.fromDegrees(120);
                else
                    targetAngle = Rotation2d.fromDegrees(60);
            }
            else if (xbox.getHID().getBackButton()) {
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                    targetAngle = Rotation2d.fromDegrees(60);
                else
                    targetAngle = Rotation2d.fromDegrees(120);
            }
            else 
                targetAngle = Rotation2d.fromDegrees(
                    targetAngle.getDegrees() -
                    xbox.getRightX() * coefficient * MAX_ROTATIONAL_SPEED
                );
            base.setSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        forward,
                        sideways,
                        calculateRotationalVelocityToTarget(targetAngle)
                    ), 
                    base.getPose().getRotation()
                )
            );
        }).beforeStarting(() -> targetAngle = base.getPose().getRotation());
    }

    private double calculateRotationalVelocityToTarget(Rotation2d targetRotation) {
		double rotationalVelocity = anglePID.calculate(
			base.getPose().getRotation().getRadians(),
			targetRotation.getRadians()
		);
		if (anglePID.atSetpoint()) {
			rotationalVelocity = 0;
		}
		return rotationalVelocity;
	}
}
