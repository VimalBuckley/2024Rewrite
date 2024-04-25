package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterSim extends Shooter {
    private final TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);
    private State state = new State();
    private double shooterVolts = 0;
    private double loaderVolts = 0;
    private Supplier<Translation2d> robotTranslation;

    protected ShooterSim(Supplier<Translation2d> robotTranslation) {
        this.robotTranslation = robotTranslation;
    }

    @Override
    public Command aim() {
        return tilt(() -> TILT_CALCULATOR.get(robotTranslation.get().getDistance(
            new Translation2d(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -0.5 : 16.5, 5.9)
        ))).repeatedly();
    }

    @Override
    public Command tilt(DoubleSupplier targetTilt) {
        return Commands.run(
            () -> {
                state = profile.calculate(
                    0.02, 
                    state, 
                    new State(targetTilt.getAsDouble(), 0)
                );
            }, this
        ).until(
            () -> Math.abs(targetTilt.getAsDouble() - state.position) < TILT_TOLERANCE
        );
    }

    @Override
    public Command fire(double voltage) {
        return Commands.runOnce(() -> shooterVolts = voltage);
    }

    @Override
    public Command load(double voltage) {
        return Commands.runOnce(() -> loaderVolts = voltage);
    }

    @Override
    public ShooterState getState() {
        return new ShooterState(state.position, shooterVolts, loaderVolts);
    }
}