/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonFX intake_master = new WPI_TalonFX(1);
  // private WPI_TalonSRX intake_slave = new WPI_TalonSRX(1);

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    // intake_slave.follow(intake_master);
  }

  public void setIntakeSpeed(double speed){
    intake_master.set(speed);
    SmartDashboard.putNumber("FalconSpeed1", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
