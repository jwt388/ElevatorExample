package frc.sim;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ElevatorConstants;

/** Constants utility class for the simulation. */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  /** Elevator simulation constants. */
  public static final class ElevatorSimConstants {
    private ElevatorSimConstants() {
      throw new IllegalStateException("ElevatorSimConstants Utility Class");
    }

    public static final double ELEVATOR_REDUCTION = ElevatorConstants.GEAR_RATIO;
    public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(0.5);
    public static final double CARRIAGE_MASS = 28.4; // kg
  }
}
