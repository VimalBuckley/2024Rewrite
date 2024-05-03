package frc.robot.subsystems.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.subsystems.telescope.TelescopeConstants.*;

public class TelescopeSim extends TelescopeIO {
    private State state = new State();
    private TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);

    @Override
    public Command extend(double extension) {
        var target = new State(extension, 0);
        return Commands.run(() -> {
            state = profile.calculate(0.02, state, target);
        }).until(() -> Math.abs(state.position - target.position) < EXTENSION_TOLERANCE)
            .finallyDo(() -> state = new State(state.position, 0));
    }

    @Override
    public TelescopeState getState() {
        return new TelescopeState(state.position);
    }
    
}
