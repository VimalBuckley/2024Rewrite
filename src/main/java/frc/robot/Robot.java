// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    @Override
    public void robotInit() {
        initLogging();
        new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    private void initLogging() {
        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        } else {
            Logger.addDataReceiver(new WPILOGWriter("logs")); // Log to the logs folder in the robot code project
        } 
		Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
		Logger.start();
	}
}
