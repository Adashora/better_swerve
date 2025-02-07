// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.subsystems.Swerve_module;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

public static final double robot_length = Units.inchesToMeters(25);
public static final double robot_width = Units.inchesToMeters(25);

public static final Translation2d front_right = new Translation2d(robot_length/2, -robot_width/2);
public static final Translation2d front_left = new Translation2d(robot_length/2, robot_width/2);
public static final Translation2d back_right = new Translation2d(-robot_length/2, -robot_width/2);
public static final Translation2d back_left =  new Translation2d(-robot_length/2, robot_width/2);





public static final SwerveDriveKinematics Swerve_map = new SwerveDriveKinematics(
  front_right,
  back_right,
  back_left,
  front_left
 );



  

  public final class dt{
    

    public class Module_0{
      
      public static final int module_number = 0;
      public static final int drive_id = 0;
      public static final int turn_id = 0;
      public static final int Cancoder_id = 0;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(0);

    }
    public class Module_1{
      //public static final int module_number = 1;
      public static final int drive_id = 0;
      public static final int turn_id = 0;
      public static final int Cancoder_id = 0;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(0);

    }
    public class Module_2{
     // public static final int module_number = 2;
      public static final int drive_id = 0;
      public static final int turn_id = 1;
      public static final int Cancoder_id = 0;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(0.0);

    }
    public class Module_3{
      
     // public static final int module_number = 3;
      public static final int drive_id = 0;
      public static final int turn_id = 0;
      public static final int Cancoder_id = 0;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(0.0);

    }

  }

public static final double max_angular_speed = 0.0;

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

    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }
  }
  

