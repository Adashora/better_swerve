// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public PigeonIMU gyro;

  public Swerve_module[] hehe;

  /** Creates a new Drivetrain. */
  public Drivetrain() {


    this.hehe = new Swerve_module[] { // sets up the swerve module instances in an array

        new Swerve_module(0, Constants.dt.Module_0.drive_id, Constants.dt.Module_0.turn_id, Constants.dt.Module_0.Cancoder_id,
            Constants.dt.Module_0.turn_offset),
        new Swerve_module(1, Constants.dt.Module_1.drive_id, Constants.dt.Module_1.turn_id, Constants.dt.Module_1.Cancoder_id,
            Constants.dt.Module_1.turn_offset),
        new Swerve_module(2, Constants.dt.Module_2.drive_id, Constants.dt.Module_2.turn_id, Constants.dt.Module_2.Cancoder_id,
            Constants.dt.Module_2.turn_offset),
        new Swerve_module(3, Constants.dt.Module_3.drive_id, Constants.dt.Module_3.turn_id, Constants.dt.Module_3.Cancoder_id,
            Constants.dt.Module_3.turn_offset)
    };

    gyro = new PigeonIMU(10); // defines gyro

    gyro.configFactoryDefault(); // gyro config

    set_gyro(0);

  }

  public void set_gyro(double yaw) { // sets gyro
    gyro.setYaw(yaw);
  }

  public Rotation2d get_yaw() { // gets yaw

    return Rotation2d.fromDegrees(gyro.getYaw());

  }

  public void drive(Translation2d translation, double rotation, boolean isfieldrelative) {

    SwerveModuleState[] states = Constants.Swerve_map.toSwerveModuleStates( /* if field relatie is true converts sped
                                                                            using yaw angle */
        isfieldrelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, get_yaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.max_speed); // no wheel goes faster than max

    for (Swerve_module module : this.hehe) {
      module.set_desired_state(states[module.module_number]);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
