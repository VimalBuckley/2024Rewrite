// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
        xbox.a().onTrue(swerve.resetGyro());
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(chooser);
        RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
    }
}
