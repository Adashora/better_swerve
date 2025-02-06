// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class Swerve_module {
public  SparkMax drive_motor;
public  SparkMax turn_motor;
public  double module_number;

public double turn_speed;
public double drive_speed;
PIDController turn_PID;
PIDController drive_PID;

Rotation2d turn_offset;

private final CANCoder best_turnEncoder;
private AbsoluteSensorRange range;
private final RelativeEncoder drive_encoder;
private final RelativeEncoder turn_encoder;

private final SparkMaxConfig drive_config;
private final SparkMaxConfig turn_config;

    public Swerve_module(int module_number, int drive_motor_ID, int turn_motor_ID, int cancoder_ID, Rotation2d turn_offset) {
        this.turn_offset = turn_offset;
        this.drive_config = new SparkMaxConfig();
        this.turn_config = new SparkMaxConfig();
    
       this.module_number = module_number;
      

        
      
       
       
       
       //cancoder stuff
       this.best_turnEncoder = new CANCoder(cancoder_ID); 
       range = AbsoluteSensorRange.Signed_PlusMinus180; 
        this.best_turnEncoder.configAbsoluteSensorRange(range); //sets the range of the encoder
     
     
       //drive motors stuff
        this.drive_motor = new SparkMax(drive_motor_ID, MotorType.kBrushless); //defines drive motor
        this.drive_encoder = this.drive_motor.getEncoder(); //gerts the encoder fomr the dirve motor
    
        drive_config
            .voltageCompensation(12)
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        drive_config.encoder
            .positionConversionFactor(Constants.dt.position_conversion_factor)
            .velocityConversionFactor(Constants.dt.velocity_conversion_factor);
         //sets the current limit to 40 amps
        

        this.drive_motor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        drive_PID = new PIDController(Constants.dt.drivekp, Constants.dt.driveki, Constants.dt.drivekd);


        //turn motor stuff
        turn_motor = new SparkMax(turn_motor_ID, MotorType.kBrushless); //defines turn motor
        this.turn_encoder = this.turn_motor.getEncoder(); //gets the encoder from the turn motor
        
        
        best_turnEncoder.configAbsoluteSensorRange(range);

        turn_config
            .voltageCompensation(12)
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        turn_config.encoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360 / 60);
        turn_config.signals
            .primaryEncoderPositionPeriodMs(500);
          
            this.turn_motor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
            

       turn_PID = new PIDController(Constants.dt.turnKP, Constants.dt.turnKI, Constants.dt.turnKD);
      


    }

    public Rotation2d getCANCoder() {
        double angle = this.best_turnEncoder.getAbsolutePosition();
        return Rotation2d.fromDegrees(angle);
    }

    public SwerveModuleState getState() {
        double speedMetersPerSecond = this.drive_encoder.getVelocity(); // * Constants.dt.velocity_conversion_factor;
        Rotation2d angle = getCANCoder();
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState current_state = this.getState();


        Rotation2d current_rotation = current_state.angle.minus(this.turn_offset).plus(new Rotation2d());

       desiredState.optimize(current_rotation);

       desiredState.cosineScale(current_rotation);

       Rotation2d desired_rotation = desiredState.angle.plus(new Rotation2d());

       double diff = desired_rotation.getDegrees() - current_rotation.getDegrees();

       if (Math.abs(diff) < 1){
              turn_speed = 0;
       }
         else{
              turn_speed = turn_PID.calculate(diff, 0);
         }


         drive_speed = desiredState.speedMetersPerSecond/Constants.dt.max_speed;
        
        
         turn_motor.set(turn_speed);
        drive_motor.set(drive_speed);
        
    }

}

