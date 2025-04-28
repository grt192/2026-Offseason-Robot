// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveDriveConstants {

    // Motor Constants (DRIVE)
    public static final double DRIVE_PEAK_CURRENT = 80; // Maximum current limit for the motor in amps, this number is in CTRE docs 
    public static final double DRIVE_RAMP_RATE = 0; // Time in seconds for the motor to go from neutral to full throttle NEED TO TUNE

    // Physical Measurements (DRIVE)
    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4 * Math.PI); // Circumference of the drive wheel in Meters
    public static final double DRIVE_GEAR_REDUCTION = 180. / 26.; // Gear reduction ratio  for Drive


  }
  public static class SwerveSteerConstants{
      // Motor Constants (STEER)
      public static final double STEER_PEAK_CURRENT = 80; // Maximum current limit for the motor in amps, this number is in CTRE docs 
      public static final double STEER_RAMP_RATE = 0; // Time in seconds for the motor to go from neutral to full throttle NEED TO TUNE
  
      // Physical Measurements (STEER)
      public static final double STEER_GEAR_REDUCTION = 14.8; // Gear reduction ratio for steer
  

  }
}
