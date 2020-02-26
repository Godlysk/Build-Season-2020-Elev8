/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DifferentialDriveSubsystem extends SubsystemBase {

  public DifferentialDrive drive;

  public AHRS navX;

  private double rcw;

  public static final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);
  public static final WPI_TalonSRX rightSlave = new WPI_TalonSRX(4);;
  public static final WPI_TalonSRX leftMaster = new WPI_TalonSRX(2);;
  public static final WPI_TalonSRX leftSlave = new WPI_TalonSRX(1);;

  // The motors on the left side of the drive.
  //private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(new WPI_TalonSRX(Constants.FL_port),new WPI_TalonSRX(Constants.BL_port));

  // The motors on the right side of the drive.
  //private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new WPI_TalonSRX(Constants.FR_port),new WPI_TalonSRX(Constants.BR_port));

  /**
   * Creates a new DifferentialDriveSubsystem.
   */

  public DifferentialDriveSubsystem(final AHRS navX) {
    drive = new DifferentialDrive(leftMaster, rightMaster);
    this.navX = navX;
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    rightMaster.setInverted(true);
    leftMaster.setInverted(false);
  }

  double integral, previous_error, setpoint = 0;

  public void setSetpoint(final int setpoint) {
    this.setpoint = setpoint;
  }

  public void PID() {
    final double error = setpoint - navX.getAngle(); // Error = Target - Actual
    this.integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
                                    // IterativeRobot)
    final double derivative = (error - this.previous_error) / .02;
    this.rcw = Constants.kP * error + Constants.kI * this.integral + Constants.kD * derivative;
  }

  public void execute() {

  }

  public DifferentialDriveKinematics gDifferentialDriveKinematics() {
    return gDifferentialDriveKinematics();
  }

  public void drive(final double yaxis, final double zaxis) {
    PID();
    drive.arcadeDrive(yaxis * Constants.maxSpeed, (zaxis + rcw) * Constants.maxSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}