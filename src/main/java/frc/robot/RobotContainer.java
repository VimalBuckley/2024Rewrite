// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.telescope.Telescope;

public class RobotContainer {
    private final Mechanism2d robotMech = new Mechanism2d(1.372, 1.2192);
    private final Swerve swerve = Swerve.getInstance();
    private final Shooter shooter = Shooter.getInstance(() -> swerve.getState().pose().getTranslation());
    private final Telescope telescope = Telescope.getInstance();
    private final CommandXboxController xbox = new CommandXboxController(2);
    private final CommandJoystick stick = new CommandJoystick(1);
    public RobotContainer() {
        setupMechanism2d();
        setupLogging();
        setupBindings();
        setupAuto();
    }

    private void setupMechanism2d() {
        robotMech.getRoot("Telescope Root", 0.483, 0.1524).append(telescope.mech);
        telescope.mech.append(shooter.mech);
    }

    private void setupBindings() {
        swerve.setDefaultCommand(swerve.angleCentric(xbox));
        xbox.a().onTrue(swerve.resetGyro());
        xbox.rightTrigger().whileTrue(AutoBuilder.pathfindToPoseFlipped(
            new Pose2d(1.8, 7.8, Rotation2d.fromDegrees(-90)), SwerveConstants.TELEOP_CONSTRAINTS));

        stick.button(1); //shoot
        stick.button(2); //intake
        stick.button(3).onTrue(variableShot()); // variable shot
        stick.button(5); // amp
        stick.button(6); // ferry
        stick.button(7); // finish climb
        stick.button(8); // start climb
        stick.button(9); // baby bird
        stick.button(10).onTrue(stow()); // stow
        stick.povRight(); // ready subwoofer
        stick.button(11); // eject
        stick.button(12); // backout
    }

    private void setupLogging() {
        new Timer().schedule(
			new TimerTask() {
				public void run() {
					Logger.processInputs("Swerve", swerve);
                    Logger.processInputs("Shooter", shooter);
                    Logger.processInputs("Telescope", telescope);
                    Logger.recordOutput("State", robotMech);
				}
			},
			10,
			20
		);
    }

    private void setupAuto() {
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(chooser);
        RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
    }

    private Command stow() {
        return shooter.fire(0)
            .andThen(shooter.load(0))
            .andThen(shooter.tilt(11.5))
            .alongWith(telescope.extend(0));
    }

    private Command variableShot() {
        return shooter.fire(12)
            .andThen(shooter.aim())
            .alongWith(telescope.extend(0.5));
    }
}
