// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Set;

/** Utility class for managing preferences. */
// TODO - Add functions to manage preferences more automatically with lists of keys and
//        default values defined in the constants file.
public class RobotPreferences {

  private RobotPreferences() {
    throw new IllegalStateException("RobotPreferences Utility class");
  }

  /** Reset the Preferences table to default values. */
  public static void resetPreferences() {}

  /** Log the values of all entries in the Preferences table. */
  public static void logPreferences() {
    NetworkTable prefTable = NetworkTableInstance.getDefault().getTable("Preferences");
    Set<String> prefKeys = prefTable.getKeys();

    for (String keyName : prefKeys) {
      NetworkTableType prefType = prefTable.getEntry(keyName).getType();

      if (prefType == NetworkTableType.kDouble) {
        DataLogManager.log(
            "Preferences/" + keyName + ": " + prefTable.getEntry(keyName).getDouble(-1));
      } else if (prefType == NetworkTableType.kBoolean) {
        DataLogManager.log(
            "Preferences/" + keyName + ": " + prefTable.getEntry(keyName).getBoolean(false));
      }
    }
  }
}
