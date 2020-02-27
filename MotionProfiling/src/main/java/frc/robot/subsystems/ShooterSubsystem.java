/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonSRX shooter = new WPI_TalonSRX(1);
  private final WPI_TalonSRX intake_master = new WPI_TalonSRX(2);
  private final WPI_TalonSRX intake_slave = new WPI_TalonSRX(3);

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    intake_slave.follow(intake_master);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed){
    intake_master.set(speed*Constants.maxIntakeSpeed);
  }

  public void setShooterSpeed(double speed){
    shooter.set(speed*Constants.maxShooterSpeed);
  }
  
}
