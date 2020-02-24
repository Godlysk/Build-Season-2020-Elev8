/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final SpeedControllerGroup rightSide;

  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  
  private final SpeedControllerGroup leftSide;

  private final DifferentialDrive drive;
  private final SimpleMotorFeedforward feedforward;
  
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    
    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    rightSide = new SpeedControllerGroup(FR, BR);
    
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port); 
    leftSide = new SpeedControllerGroup(FL, BL);
    
    drive = new DifferentialDrive(leftSide, rightSide);
    feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void arcadeInbuilt(double y, double z) {
    FR.setInverted(false);
    BR.setInverted(false);

    drive.arcadeDrive(y * Constants.maxSpeed, z * Constants.maxSpeed);
  }

  public void arcadeInbuilt(double y, double z, boolean s) {
    FR.setInverted(false);
    BR.setInverted(false);

    drive.arcadeDrive(y * Constants.maxSpeed, z * Constants.maxSpeed, s);
  }

  public void tankFeedForward(double lvel, double rvel, double pidOutputs) {
    FR.setInverted(true);
    BR.setInverted(true);

    FL.setVoltage(feedforward.calculate(lvel) - pidOutputs);
    BL.setVoltage(feedforward.calculate(lvel) - pidOutputs);
    FR.setVoltage(feedforward.calculate(rvel) + pidOutputs);
    BR.setVoltage(feedforward.calculate(rvel) + pidOutputs);
  }

  public void curvatureInbuilt(double y, double z) {
    // Do Nothing for now
  }

  // Needs work
  public void stopDrive() {
    tankFeedForward(0, 0, 0);
  }

  // Utility
  public void drive(double l, double r) {
    FR.setInverted(true);
    BR.setInverted(true);

    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }


  // Inbuilt stuff
  double integral, prevError = 0;

  public void drivePID(double yaxis) {

    if (Math.abs(yaxis) < Constants.integralResetBound) integral = 0;

    double leftRate = RobotContainer.enc_L.getRate();
    double rightRate = RobotContainer.enc_R.getRate();

    double error = leftRate - rightRate;
    integral += (error * Constants.kI_custom);

    double derivative = (error - prevError) * Constants.kD_custom;
    prevError = error; 

    double correction = (error * Constants.kP_custom);
    correction += (integral);
    correction += (derivative);

    double left = yaxis - correction;
    double right = yaxis + correction;

    drive(left * Constants.maxSpeed, right * Constants.maxSpeed);

  }

  public void steerCustom(double yaxis, double zaxis) {

    double steer = zaxis * Constants.steerCoefficient;
    double left = yaxis * Constants.maxSpeed + steer;
    double right = yaxis * Constants.maxSpeed - steer;
    
    drive(left, right);
    
  }


  // Vision Function
  public void followBall(double yaxis, int cX, int radius) {

    double error = (double) (Constants.centreScreenX - cX);
    SmartDashboard.putNumber("Error Vision", error);

    if (cX != -1 && radius != -1) 
      arcadeInbuilt(yaxis * (15/radius), -error * Constants.kP_balls, false);
    else arcadeInbuilt(0, 0);

  }




}
