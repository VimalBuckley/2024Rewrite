package frc.robot.subsystems.telescope;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TelescopeIO extends SubsystemBase implements LoggableInputs {
    private static TelescopeIO instance;
    public static synchronized TelescopeIO getInstance() {
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
    public TelescopeIO() {
        mech = new MechanismLigament2d("Telescope", 0.4064, 105);
    }

    public abstract Command extend(double extension);
    public abstract TelescopeState getState();

    @Override
    public void periodic() {
        mech.setLength(0.4064 + getState().extension());
    }

    public void toLog(LogTable table) {
        table.put("Extension", getState().extension());
    }

    public void fromLog(LogTable table) {}

    public static record TelescopeState(double extension) {}
}
