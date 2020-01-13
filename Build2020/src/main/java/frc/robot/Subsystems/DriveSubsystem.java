/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port);  

    FR.setInverted(true);
    BR.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void drive(double leftSpeed, double rightSpeed) {
    FR.set(rightSpeed);
    BR.set(rightSpeed);
    FL.set(leftSpeed);
    BL.set(leftSpeed);
  }


  public void Drive_Straight(double yaxis) {
    double left = yaxis;
    double right = yaxis;
    drive(left*Constants.maxSpeed, right*Constants.maxSpeed);
  }

  public void Drive_Steer(double yaxis, double zaxis) {
    double error = zaxis*Constants.swerveCoefficient;   
    double left = yaxis + error;
    double right = yaxis - error;
    drive(left*Constants.maxSpeed, right*Constants.maxSpeed);

  }

  public void Drive_Turn(double zaxis) {

    double left = zaxis;
    double right = -1 * (zaxis);
    drive(left*Constants.maxSpeed, right*Constants.maxSpeed);
    
  }



}
