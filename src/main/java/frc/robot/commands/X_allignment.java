// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Vision;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class X_allignment extends Command {

//   Vision vision;
//   Drivetrain drivetrain;
//   SlewRateLimiter x_limit = new SlewRateLimiter(3);

//   double x_offset;
//   double x;
//   PIDController PID_vision;



//   /** Creates a new X_allignment. */
//   public X_allignment(Vision vision, Drivetrain drivetrain) {

//     this.x_offset = vision.get_x_offset();
//     this.vision = vision;
//     this.drivetrain = drivetrain;

//     PID_vision = new PIDController(Constants.vision.vision_p, Constants.vision.vision_i, Constants.vision.vision_d);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }
//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     if (this.vision.See_tag()){
//       x = x_limit.calculate(PID_vision.calculate(x_offset, 0));

//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
