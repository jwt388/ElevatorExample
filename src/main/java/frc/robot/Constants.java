// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to enable using Tunable Numbers
  public static final boolean TUNING_MODE = true;

  // Set to true to log each frame of command execution. To false to disable.
  public static final boolean COMMAND_EXECUTE_LOG = false;

  /** Constants used for the Elevator subsystem. */
  public static final class ElevatorConstants {

    private ElevatorConstants() {
      throw new IllegalStateException("ElevatorConstants Utility Class");
    }

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final int MOTOR_PORT = 8;
    public static final int CURRENT_LIMIT = 40;

    // Constants tunable through TunableNumbers
    public static final double ELEVATOR_KP = 15.0;
    public static final double ELEVATOR_KS = 0.1;
    public static final double ELEVATOR_KG = 0.65;
    public static final double ELEVATOR_KV_VOLTS_PER_METER_PER_SEC = 24.55;
    public static final double ELEVATOR_MAX_VELOCITY_METERS_PER_SEC = 0.5;
    public static final double ELEVATOR_MAX_ACCELERATION_METERS_PER_SEC2 = 2.0;

    public static final double GEAR_RATIO = 32.0;
    public static final double ELEVATOR_METERS_PER_ENCODER_ROTATION = 2.0 * Math.PI / GEAR_RATIO;
    public static final double RPM_TO_METERS_PER_SEC = ELEVATOR_METERS_PER_ENCODER_ROTATION / 60;
    public static final double ELEVATOR_HIGH_POSITION = 0.8;
    public static final double ELEVATOR_LOW_POSITION = 0.2;
    public static final double ELEVATOR_OFFSET_RADS = 0.0;

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double ELEVATOR_MIN_HEIGHT_METERS = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT_METERS = 1.25;

    public static final double POSITION_TOLERANCE_METERS = 0.03;
    public static final double VELOCITY_TOLERANCE_METERS = 0.01;
  }

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}
