package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.EZLogger.LogAccess;
import frc.robot.utilities.EZLogger.Loggable;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class ShooterIO extends SubsystemBase implements LoggableInputs, Loggable {
    private static ShooterIO instance;
    public static synchronized ShooterIO getInstance(Supplier<Translation2d> pose) {
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
    public ShooterIO() {
        mech = new MechanismLigament2d("Shooter", 0.3, -38);
    }

    public abstract Command aim();
    public abstract Command tilt(double targetTilt);
    public abstract Command fire(double voltage);
    public abstract Command load(double voltage);
    public abstract ShooterState getState();
    
    public void periodic() {
        mech.setAngle(getState().shooterTilt() - 105);
    }

    public void toLog(LogTable table) {
        table.put("Tilt", getState().shooterTilt());
        table.put("Shooter Voltage", getState().shooterVoltage());
        table.put("Loader Voltage", getState().loaderVoltage());
    }

    public void log(LogAccess table) {
        table.put("Tilt", getState().shooterTilt());
        table.put("Shooter Voltage", getState().shooterVoltage());
        table.put("Loader Voltage", getState().loaderVoltage());
    }
    
    public void fromLog(LogTable table) {}

    public static record ShooterState(double shooterTilt, double shooterVoltage, double loaderVoltage) {}
}
