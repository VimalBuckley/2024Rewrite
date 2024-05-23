package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.EZLogger.LogAccess;
import frc.robot.utilities.EZLogger.Loggable;

public abstract class IntakeIO extends SubsystemBase implements LoggableInputs, Loggable {
    private static IntakeIO instance;
    public static synchronized IntakeIO getInstance() {
        if (instance == null) {
            if (RobotBase.isReal()) {
                instance = null;
            } else {
                instance = new IntakeSim();
            }
        }
        return instance;   
    }

    public final MechanismLigament2d mech;
    public IntakeIO() {
        mech = new MechanismLigament2d("Intake", 0.3, 45);
    }

    public abstract Command tilt(double target);
    public abstract Command spin(double voltage);
    public abstract IntakeState getState();

    @Override
    public void periodic() {
        mech.setAngle(getState().tilt());
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Tilt", getState().tilt());
        table.put("Voltage", getState().voltage());
    }

    @Override
    public void log(LogAccess table) {
        table.put("Tilt", getState().tilt());
        table.put("Voltage", getState().voltage());
    }
    
    @Override
    public void fromLog(LogTable table) {}

    public static record IntakeState(double tilt, double voltage) {}
}
