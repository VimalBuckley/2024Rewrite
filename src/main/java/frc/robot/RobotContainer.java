// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.telescope.TelescopeIO;
import frc.robot.utilities.CommandKeyboard;
import frc.robot.utilities.EZLogger;
import frc.robot.utilities.EZLogger.Loggable;
import frc.robot.subsystems.telescope.TelescopeConstants;

public class RobotContainer {
    private final Mechanism2d robotMech = new Mechanism2d(1.372, 1.2192);
    private final SwerveIO swerve = SwerveIO.getInstance();
    private final ShooterIO shooter = ShooterIO.getInstance(() -> swerve.getState().pose().getTranslation());
    private final TelescopeIO telescope = TelescopeIO.getInstance();
    private final IntakeIO intake = IntakeIO.getInstance();
    private final CommandXboxController xbox = new CommandXboxController(2);
    private final CommandJoystick stick = new CommandJoystick(1);
    private final SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    public RobotContainer() {
        setupMechanism2d();
        setupLogging();
        setupBindings();
        setupAuto();
    }

    private void setupMechanism2d() {
        robotMech.getRoot("Intake Root", 0.972, 0.1).append(intake.mech);
        robotMech.getRoot("Telescope Root", 0.483, 0.1524).append(telescope.mech);
        telescope.mech.append(shooter.mech);
        // Rotation2d testRotation = Rotation2d.fromDegrees(180);

        // StructBuffer<Rotation2d> testBuffer = StructBuffer.create(new Rotation2dStruct());
        // ByteBuffer bytes = testBuffer.write(testRotation);
        // byte[] array = new byte[bytes.position()];
        // bytes.position(0);
        // bytes.get(array); 
        // SmartDashboard.putRaw("Test Rotation", array);
        // Logger.recordOutput("Test Rotation", Rotation2d.fromDegrees(180));
    }

    private void setupBindings() {
        swerve.setDefaultCommand(swerve.angleCentric(xbox));
        xbox.a().onTrue(swerve.resetGyro());
        xbox.rightTrigger().whileTrue(AutoBuilder.pathfindToPoseFlipped(
            new Pose2d(1.8, 7.8, Rotation2d.fromDegrees(-90)), SwerveConstants.TELEOP_CONSTRAINTS));
        xbox.povUp().onTrue(startIntaking()).onFalse(stow());
        xbox.povDown().onTrue(variableShot());
        xbox.leftStick().onTrue(stow());
        xbox.povRight().onTrue(readyAmp());

        stick.button(1).onTrue(shoot());
        stick.button(2).onTrue(startIntaking()); //intake
        stick.button(3).onTrue(variableShot()); // variable shot
        stick.button(5).onTrue(readyAmp()); // amp
        stick.button(6); // ferry
        stick.button(7); // finish climb
        stick.button(8); // start climb
        stick.button(9); // baby bird
        stick.button(10).onTrue(stow()); // stow
        stick.povRight(); // ready subwoofer
        stick.button(11); // eject
        stick.button(12); // backout

        CommandKeyboard.getKey("t")
            .onTrue(Commands.print("t was pressed :)"))
            .onFalse(Commands.print("t was released :("));
    }

    private void setupLogging() {
        EZLogger.put("Swerve", (Loggable) swerve);
        EZLogger.put("Shooter", (Loggable) shooter);
        EZLogger.put("Telescope", (Loggable) telescope);
        EZLogger.put("Intake", (Loggable) intake);
        EZLogger.put("State", robotMech);
        EZLogger.put("Auto Chooser", chooser);
    }

    private void setupAuto() {
        RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
    }

    private Command stow() {
        return shooter.fire(ShooterConstants.REST_VOLTAGE)
            .andThen(shooter.load(ShooterConstants.LOADER_REST_VOLTAGE))
            .andThen(intake.spin(IntakeConstants.REST_VOLTAGE))
            .andThen(shooter.tilt(ShooterConstants.STOW_TILT))
            .alongWith(telescope.extend(TelescopeConstants.STOW_EXTENSION))
            .alongWith(intake.tilt(IntakeConstants.STOW_TILT));
    }

    private Command shoot() {
        return shooter.load(ShooterConstants.LOADER_SHOT_VOLTAGE)
            .andThen(Commands.waitSeconds(0.9))
            .andThen(stow());
    }

    private Command variableShot() {
        return shooter.fire(ShooterConstants.SHOT_VOLTAGE)
            .andThen(shooter.aim())
            .alongWith(telescope.extend(TelescopeConstants.SPEAKER_EXTENSION));
    }

    private Command startIntaking() {
        return shooter.tilt(ShooterConstants.STOW_TILT)
            .alongWith(telescope.extend(TelescopeConstants.STOW_EXTENSION))
            .andThen(intake.spin(IntakeConstants.INTAKE_VOLTAGE))
            .andThen(intake.tilt(IntakeConstants.GROUND_TILT));
    }

    private Command readyAmp() {
        return shooter.load(ShooterConstants.LOADER_REST_VOLTAGE)
            .andThen(shooter.fire(ShooterConstants.AMP_VOLTAGE))
            .andThen(telescope.extend(TelescopeConstants.AMP_EXTENSION))
            .alongWith(shooter.tilt(ShooterConstants.AMP_TILT));
    }
}
