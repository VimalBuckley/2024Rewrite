// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.telescope.Telescope;

public class RobotContainer {
    private final Mechanism2d robotMech = new Mechanism2d(1.372, 1.2192);
    private final Swerve swerve = Swerve.getInstance();
    private final Shooter shooter = Shooter.getInstance(() -> swerve.getState().pose().getTranslation());
    private final Telescope telescope = Telescope.getInstance();
    private final CommandXboxController xbox = new CommandXboxController(2);
    public RobotContainer() {
        robotMech.getRoot("Telescope Root", 0.483, 0.1524).append(telescope.mech);
        telescope.mech.append(shooter.mech);
        swerve.setDefaultCommand(swerve.angleCentric(xbox));
        SmartDashboard.putData("Robot Mech", robotMech);
        new Timer().schedule(
			new TimerTask() {
				public void run() {
					Logger.processInputs("Swerve", swerve);
                    Logger.processInputs("Shooter", shooter);
                    Logger.recordOutput("State", robotMech);
				}
			},
			10,
			20
		);
        xbox.a().onTrue(swerve.resetGyro());
        xbox.b().whileTrue(shooter.aim());
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(chooser);
        RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
    }
}
