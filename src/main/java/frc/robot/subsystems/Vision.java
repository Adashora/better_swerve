// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
    NetworkTable network_table; 
    NetworkTableEntry tx; 
    NetworkTableEntry ty;     //sets variables
    NetworkTableEntry ta;  
    NetworkTableEntry priority_id; 
    PIDController vision_PID;
    

  public Vision() {

      this.network_table = NetworkTableInstance.getDefault().getTable("limelight");
     
      
      this.tx = network_table.getEntry("tx");
      this.priority_id = network_table.getEntry("priority ID"); 
      this.ty = network_table.getEntry("ty"); 
      this.ta = network_table.getEntry("ta");
      

      this.vision_PID = new PIDController(Constants.vision.vision_p, Constants.vision.vision_i, Constants.vision.vision_d);
      
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);  
      double area = ta.getDouble(0.0);
  

      SmartDashboard.putNumber("LimelightX", x);    //puts on smart dashboard
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("LimelightArea", area);
  
  
  
    }

  @Override
  public void periodic() {



    
    // This method will be called once per scheduler run
  }
}
