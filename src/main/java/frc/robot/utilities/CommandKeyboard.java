package frc.robot.utilities;
/**
 * Reads keyboard inputs sent over NT from a python program, and converts them to a controller
 */

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandKeyboard {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Keyboard");
    public static Trigger getKey(String key) {
        BooleanEntry entry = table.getBooleanTopic(key).getEntry(false);
        return new Trigger(() -> entry.get());
    }
}
