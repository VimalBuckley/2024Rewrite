package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.shooter.ShooterConstants.OFFSET_FROM_INTUITIVE;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class Shooter extends SubsystemBase implements LoggableInputs {
    private static Shooter instance;
    public static synchronized Shooter getInstance(Supplier<Translation2d> pose) {
        if (instance == null) {
            if (RobotBase.isReal()) {
                instance = null;
            } else {
                instance = new ShooterSim(pose);
            }
        }
        return instance;   
    }

    public final MechanismLigament2d mech;
    public Shooter() {
        mech = new MechanismLigament2d("Shooter", 0.3, -38);
    }

    public abstract Command aim();
    public abstract Command tilt(DoubleSupplier targetTilt);
    public abstract Command fire(double voltage);
    public abstract Command load(double voltage);
    public abstract ShooterState getState();
    
    public void periodic() {
        mech.setAngle(getState().shooterTilt() - 105 - OFFSET_FROM_INTUITIVE);
    }

    public void toLog(LogTable table) {
        table.put("Tilt", getState().shooterTilt());
        table.put("Shooter Voltage", getState().shooterVoltage());
        table.put("Loader Voltage", getState().loaderVoltage());
    }
    public void fromLog(LogTable table) {}

    public static record ShooterState(double shooterTilt, double shooterVoltage, double loaderVoltage) {}
}
