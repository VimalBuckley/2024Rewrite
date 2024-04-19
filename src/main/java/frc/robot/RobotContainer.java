// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
    private Swerve swerve;
    private CommandXboxController xbox;
    public RobotContainer() {
        swerve = new Swerve();
        xbox = new CommandXboxController(2);
        swerve.setDefaultCommand(swerve.angleCentric(xbox));
        new Timer().schedule(
			new TimerTask() {
				public void run() {
					Logger.processInputs("Swerve", swerve);
				}
			},
			10,
			20
		);
        xbox.a().whileTrue(AutoBuilder.pathfindToPoseFlipped(new Pose2d(4, 6, new Rotation2d()), new PathConstraints(5, 4, 9.425, 12.5664)));
    }
}
