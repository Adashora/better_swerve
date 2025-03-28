// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  Translation2d trans;
  Drivetrain dt;
  Joystick joystickR;
  Joystick joystickL;

  double dtx;
  double dty; 

  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3); // speed liimits for x y and rotation
  SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(3);
  /** Creates a new Drive. */

  public Drive(Drivetrain dt, Joystick joystickR, Joystick joystickL) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.joystickR = joystickR;
    this.joystickL = joystickL;
    addRequirements(this.dt);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    dtx = -xSpeedLimiter.calculate(MathUtil.applyDeadband(this.joystickL.getY(), 0.1)); //applies 0.1 deadband to the joysticks
    dty = -xSpeedLimiter.calculate(MathUtil.applyDeadband(this.joystickL.getX(), 0.1));
    double rot = rotSpeedLimiter.calculate(MathUtil.applyDeadband(this.joystickR.getX(), 0.1)) * Constants.max_angular_speed;

    trans = new Translation2d(dtx, dty).times(Constants.max_speed);  //makes sure its not supa supa speedy

    this.dt.drive(trans, rot, true); //DRIVE!!!! 👹👹
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
