package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import java.util.function.Function;

public class Swerve extends SubsystemBase implements LoggableInputs {
    private static Swerve instance;
    public static synchronized Swerve getInstance() {
        if (instance == null) instance = new Swerve();
        return instance;
    }
    
    private SwerveBaseIO base;
    private Rotation2d targetAngle;
    private PIDController anglePID;
    private Field2d field;
    private Swerve() {
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
            new HolonomicPathFollowerConfig(MAX_MODULE_SPEED, 0.39878808909, new ReplanningConfig(true, true)), 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
            this
        );
        field = new Field2d();
        SmartDashboard.putData(field);
    }

    @Override
    public void periodic() {
        base.periodic();
        field.setRobotPose(base.getPose());
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Target Angle", targetAngle.getDegrees());
        Logger.recordOutput("Pose", base.getPose());
    }

    @Override
    public void fromLog(LogTable table) {}

    public Command fieldCentric(CommandXboxController xbox, Function<ChassisSpeeds, ChassisSpeeds> conversion) {
        return run(() -> {
            double speedCoefficient = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            double forward = 0;
            double sideways = 0;
            if (alliance == Alliance.Blue) {
                forward = speedCoefficient * -xbox.getLeftY() * MAX_FORWARD_SPEED;
                sideways = speedCoefficient * -xbox.getLeftX() * MAX_SIDEWAYS_SPEED;
            } else {
                forward = speedCoefficient * xbox.getLeftY() * MAX_FORWARD_SPEED;
                sideways = speedCoefficient * xbox.getLeftX() * MAX_SIDEWAYS_SPEED;
            }
            double rotational = Math.toRadians(10 * speedCoefficient * -xbox.getRightX() * MAX_ROTATIONAL_SPEED);
            ChassisSpeeds original = new ChassisSpeeds(forward, sideways, rotational);
            base.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                conversion.apply(original), 
                base.getPose().getRotation()
            ));
        });
    }

    public Command fieldCentric(CommandXboxController xbox) {
        return fieldCentric(xbox, speeds -> speeds);
    }

    public Command angleCentric(CommandXboxController xbox) {
        return fieldCentric(
            xbox, 
            speeds -> { 
                targetAngle = calculateTargetAngle(xbox);
                return new ChassisSpeeds(
                    speeds.vxMetersPerSecond, 
                    speeds.vyMetersPerSecond,
                    calculateRotationalVelocityToTarget(targetAngle)
                );
        }).beforeStarting(() -> targetAngle = base.getPose().getRotation());
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> {
            targetAngle = DriverStation.getAlliance()
                .orElse(Alliance.Blue) == Alliance.Blue ? 
                new Rotation2d() : Rotation2d.fromDegrees(180);
            base.setAngle(targetAngle);
        });
    }

    public SwerveState getState() {
        return new SwerveState(base.getPose(), base.getSpeeds());
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

    private Rotation2d calculateTargetAngle(CommandXboxController xbox) {
        double allianceCoefficient = 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                1 : -1;
        if (Math.abs(xbox.getRightY()) > 0.5)
            return Rotation2d.fromDegrees(90 + allianceCoefficient * 90 * Math.signum(xbox.getRightY()));
        else if (xbox.getHID().getRightStickButton()) 
            return Rotation2d.fromDegrees(-90);
        else if (xbox.getHID().getLeftBumper())
            return Rotation2d.fromDegrees(90);
        else if (xbox.getHID().getYButton())
            return Rotation2d.fromDegrees(-90 + allianceCoefficient * 30);
        else if (xbox.getHID().getXButton()) 
            return Rotation2d.fromDegrees(90 + allianceCoefficient * 30);
        else if (xbox.getHID().getBackButton()) 
            return Rotation2d.fromDegrees(90 - allianceCoefficient * 30);
        else if (xbox.getHID().getBButton())
            return new Translation2d(8 - allianceCoefficient * 8, 5.975)
                .minus(base.getPose().getTranslation()).getAngle()
                .plus(Rotation2d.fromDegrees(180));
        else 
            return Rotation2d.fromDegrees(
                targetAngle.getDegrees() -
                xbox.getRightX() * Math.max(MIN_COEFFICIENT, 1 - xbox.getLeftTriggerAxis()) * MAX_ROTATIONAL_SPEED
            );
    } 

    public static record SwerveState(Pose2d pose, ChassisSpeeds speeds) {}
}
