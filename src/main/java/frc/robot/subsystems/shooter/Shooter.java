package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class Shooter extends SubsystemBase implements LoggableInputs {
    private final Constraints constraints = new Constraints(0, 0);
    public abstract Command aim(Supplier<Pose2d> poseSupplier);
    public abstract Command tilt(double tilt);
    public abstract Command fire(double voltage);
    public abstract Command load(double voltage);
}
