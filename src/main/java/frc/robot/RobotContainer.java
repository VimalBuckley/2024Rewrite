// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.shooter.ShooterIO;

public class RobotContainer {
    private final ShooterIO shooter;
    public RobotContainer() {
        shooter = ShooterIO.getInstance();
        SmartDashboard.putData("Shooter Subsystem", shooter);
        configureBindings();
    }

    private void configureBindings() {
        var stick = new CommandJoystick(0);
        stick.button(1).onTrue(shooter.tilt(5));
        stick.button(1).onFalse(shooter.tilt(0));
        stick.button(2).onTrue(shooter.load(12).alongWith(shooter.spinUp(11)));
        stick.button(2).onFalse(shooter.load(0).alongWith(shooter.spinUp(0)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
