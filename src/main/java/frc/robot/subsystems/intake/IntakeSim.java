package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSim extends Intake {
    private State currentState;
    private double currentVoltage;
    private TrapezoidProfile profile;
    public IntakeSim() {
        currentState = new State(135, 0);
        profile = new TrapezoidProfile(CONSTRAINTS);
        currentVoltage = 0;
    }

    @Override
    public Command tilt(double target) {
        State targetState = new State(target, 0);
        return Commands.run(
            () -> {
                currentState = profile.calculate(
                    0.02,
                    currentState,
                    targetState
                );
            }, this
        ).until(
            () -> Math.abs(targetState.position - currentState.position) < TILT_TOLERANCE
        ).finallyDo(
            () -> currentState.velocity = 0
        );
    }

    @Override
    public Command spin(double voltage) {
        return Commands.runOnce(() -> currentVoltage = voltage); 
    }

    @Override
    public IntakeState getState() {
        return new IntakeState(currentState.position, currentVoltage);
    }
    
}
