package frc.robot.subsystems.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class TelescopeConstants {
    public static final Constraints CONSTRAINTS = new Constraints(0.5, 0.5);
    public static final double EXTENSION_TOLERANCE = 0.05;
    public static final double AMP_EXTENSION = 0.5;
    public static final double SPEAKER_EXTENSION = 0.5;
    public static final double CLIMBING_EXTENSION = 0.5;
    public static final double STOW_EXTENSION = 0;
}
