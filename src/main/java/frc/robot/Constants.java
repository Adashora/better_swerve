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


  public class dt{


    public static final double max_speed = 0.0;
    public static final double position_conversion_factor = 0.0;
    public static final double velocity_conversion_factor = 0.0;


    int turn_constant = 0; //ask matthew about this




    //turn PID
    public static final double turnKP = 0.0;
    public static final double turnKI = 0.0;
    public static final double turnKD = 0.0;

    //driev pid
    public static final double drivekp = 0.0;
    public static final double driveki = 0.0;
    public static final double drivekd = 0.0;

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
