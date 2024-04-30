package frc.robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class IntakeConstants {
    public static final Constraints CONSTRAINTS = new Constraints(270, 360);
    public static final double TILT_TOLERANCE = 0.5;
    public static final double INTAKE_VOLTAGE = 12;
    public static final double OUTTAKE_VOLTAGE = -12;
    public static final double REST_VOLTAGE = 0;
    public static final double STOW_TILT = 135;
    public static final double GROUND_TILT = -25;
}
