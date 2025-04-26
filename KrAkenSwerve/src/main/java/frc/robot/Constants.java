// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveConstants {

    // Motor Constants
    public static final double PEAK_CURRENT = 60; // Maximum current limit for the motor in amps 
    public static final double RAMP_RATE = 0; // Time in seconds for the motor to go from neutral to full throttle

    // Physical Measurements
    public static final double DRIVE_WHEEL_CIRCUMFERENCE = 0; // Circumference of the drive wheel 
    public static final double DRIVE_GEAR_REDUCTION = 0; // Gear reduction ratio for the drive system 

    
    
  }
}
