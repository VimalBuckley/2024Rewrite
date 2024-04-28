package frc.robot.subsystems.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class TelescopeConstants {
    public static final Constraints CONSTRAINTS = new Constraints(0.5, 0.5);
    public static final double EXTENSION_TOLERANCE = 0.05;
}
