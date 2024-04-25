package frc.robot.subsystems.telescope;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TelescopeSim extends Telescope {

    @Override
    public Command extend(double extension) {
        return Commands.none();
    }

    @Override
    public TelescopeState getState() {
        return new TelescopeState(0);
    }
    
}
