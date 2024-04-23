package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterConstants {
    public static final Constraints CONSTRAINTS = new Constraints(120, 90);
    public static final InterpolatingDoubleTreeMap angleCalculator = new InterpolatingDoubleTreeMap();
    static {
        angleCalculator.put(1.37, -5.07);
        angleCalculator.put(1.74, -12.79);
        angleCalculator.put(2.14, -23.07);
        angleCalculator.put(2.8, -28.21);
        angleCalculator.put(3.25, -30.79);
        angleCalculator.put(3.75, -33.36);
        angleCalculator.put(4.4, -35.16);
    }   
}
