package frc.robot.subsystems.telescope;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Telescope extends SubsystemBase implements LoggableInputs {
    private static Telescope instance;
    public static synchronized Telescope getInstance() {
        if (instance == null) {
            if (RobotBase.isReal()) {
                instance = null;
            } else {
                instance = new TelescopeSim();
            }
        }
        return instance;
    }

    public final MechanismLigament2d mech;
    public Telescope() {
        mech = new MechanismLigament2d("Telescope", 0.4064, 105);
    }

    public abstract Command extend(double extension);
    public abstract TelescopeState getState();

    public void toLog(LogTable table) {}

    public void fromLog(LogTable table) {}

    public static record TelescopeState(double extension) {}
}
