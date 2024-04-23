package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterSim extends Shooter {
    private final TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);
    @Override
    public Command aim(Supplier<Pose2d> poseSupplier) {
        return Commands.run(() -> {

        }, this);
    }
    @Override
    public Command tilt(double tilt) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'tilt'");
    }

    @Override
    public Command fire(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'fire'");
    }

    @Override
    public Command load(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'load'");
    }
    
    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void fromLog(LogTable table) {}

    
}
