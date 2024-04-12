package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.sim.ShooterSim;

public abstract class ShooterIO extends SubsystemBase {
    private static ShooterIO instance;
    public static ShooterIO getInstance() {
        if (instance == null && !RobotBase.isReal()) instance = new ShooterSim();
        if (instance == null && RobotBase.isReal()) instance = null;
        return instance;
    }
    
    public abstract Command spinUp(double voltage);
    public abstract Command tilt(double tilt);
    public abstract Command load(double voltage);
    public abstract double getShooterVoltage();
    public abstract double getLoaderVoltage();
    public abstract double getTilt();
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Shooter Voltage", this::getShooterVoltage, null);
        builder.addDoubleProperty("Loader Voltage", this::getLoaderVoltage, null);
        builder.addDoubleProperty("Shooter Tilt", this::getTilt, null);
    }
}
