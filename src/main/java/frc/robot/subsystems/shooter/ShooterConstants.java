package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterConstants {
    public static final Constraints CONSTRAINTS = new Constraints(120, 90);
    public static final InterpolatingDoubleTreeMap TILT_CALCULATOR = new InterpolatingDoubleTreeMap();
    public static final double FF_OFFSET = 26.5;
    static {
        TILT_CALCULATOR.put(1.37, -44.428);
        TILT_CALCULATOR.put(1.74, -36.71);
        TILT_CALCULATOR.put(2.14, -26.43);
        TILT_CALCULATOR.put(2.8, -21.29);
        TILT_CALCULATOR.put(3.25, -18.71);
        TILT_CALCULATOR.put(3.75, -16.14);
        TILT_CALCULATOR.put(4.4, -14.34);
    }   
    public static final double TILT_TOLERANCE = 1;
    public static final double STOW_TILT = 0;
    public static final double SUBWOOFER_TILT = -45;
    public static final double AMP_TILT = 60;
    public static final double SHOT_VOLTAGE = 12;
    public static final double BACKOUT_VOLTAGE = -5;
    public static final double EJECT_VOLTAGE = -12;
    public static final double AMP_VOLTAGE = 8;
    public static final double REST_VOLTAGE = 0;
    public static final double LOADER_SHOT_VOLTAGE = 12;
    public static final double LOADER_BACKOUT_VOLTAGE = -5;
    public static final double LOADER_EJECT_VOLTAGE = -12;
    public static final double LOADER_REST_VOLTAGE = 0;
}
