package frc.robot.subsystems.shooter.sim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterIO;

public class ShooterSim extends ShooterIO {
    private double shooterVoltage;
    private double loaderVoltage;
    private double shooterTilt;
    private PIDController tiltPID;
    public ShooterSim() {
        shooterVoltage = 0;
        loaderVoltage = 0;
        shooterTilt = 0;
        tiltPID = new PIDController(0.1, 0, 0);
        tiltPID.setTolerance(0.1);
    }

    @Override
    public Command spinUp(double voltage) {
        return Commands.runOnce(
            () -> shooterVoltage = voltage
        ).withName("Spin Up Shooter");
    }

    @Override
    public Command tilt(double tilt) {
        return Commands.runOnce(
            () -> tiltPID.setSetpoint(tilt)
        ).andThen(Commands.run(
            () -> shooterTilt += tiltPID.calculate(shooterTilt), 
            this
        ).until(
            () -> tiltPID.atSetpoint()
        )).withName("Tilt Shooter");
    }

    @Override
    public Command load(double voltage) {
        return Commands.runOnce(
            () -> loaderVoltage = voltage
        ).withName("Load Shooter");
    }
    
    public double getShooterVoltage() {return shooterVoltage;}
    public double getLoaderVoltage() {return loaderVoltage;}
    public double getTilt() {return shooterTilt;}
}
