package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterConstants {
    public static final Constraints CONSTRAINTS = new Constraints(120, 90);
    public static final InterpolatingDoubleTreeMap TILT_CALCULATOR = new InterpolatingDoubleTreeMap();
    public static final double OFFSET_FROM_INTUITIVE = 26.5;
    static {
        TILT_CALCULATOR.put(1.37, -17.928);
        TILT_CALCULATOR.put(1.74, -10.21);
        TILT_CALCULATOR.put(2.14, 0.07);
        TILT_CALCULATOR.put(2.8, 5.21);
        TILT_CALCULATOR.put(3.25, 7.79);
        TILT_CALCULATOR.put(3.75, 10.36);
        TILT_CALCULATOR.put(4.4, 12.16);
    }   
    public static final double TILT_TOLERANCE = 5;
}
