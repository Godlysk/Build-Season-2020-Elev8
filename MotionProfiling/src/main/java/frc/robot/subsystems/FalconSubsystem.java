/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FalconSubsystem extends SubsystemBase {

  private WPI_TalonFX motor1 = new WPI_TalonFX(Constants.falcon1Port);
  private WPI_TalonFX motor2= new WPI_TalonFX(Constants.falcon2Port);

  /**
   * Creates a new FalconSubsystem.
   */
  public FalconSubsystem() {
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed1(double speed){
    motor1.set(-speed);
    SmartDashboard.putNumber("FalconSpeed1", -speed);
    motor2.set(speed);
    SmartDashboard.putNumber("FalconSpeed2", speed);
  }

}