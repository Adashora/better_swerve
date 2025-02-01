// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class Swerve_module {
public  SparkMax drive_motor;
public  SparkMax turn_motor;
public  double module_number;
public  double module_order;
private final CANCoder best_turnEncoder;
private AbsoluteSensorRange range;
private final RelativeEncoder drive_encoder;
private final RelativeEncoder turn_encoder;

    public Swerve_module(double module_number, double module_order, int drive_motor_ID, int turn_motor_ID, int cancoder_ID) { 
       this.module_number = module_number;
       this.module_order = module_order;
       
       
       
       //cancoder stuff
       this.best_turnEncoder = new CANCoder(cancoder_ID); 
       range = AbsoluteSensorRange.Signed_PlusMinus180; 
        this.best_turnEncoder.configAbsoluteSensorRange(range); //sets the range of the encoder
     
     
       //drive motors stuff
        this.drive_motor = new SparkMax(drive_motor_ID, MotorType.kBrushless); //defines drive motor
        this.drive_encoder = this.drive_motor.getEncoder(); //gerts the encoder fomr the dirve motor
    
       
       

        //turn motor stuff
        turn_motor = new SparkMax(turn_motor_ID, MotorType.kBrushless); //defines turn motor
        this.turn_encoder = this.turn_motor.getEncoder(); //gets the encoder from the turn motor



    
   

}


}
