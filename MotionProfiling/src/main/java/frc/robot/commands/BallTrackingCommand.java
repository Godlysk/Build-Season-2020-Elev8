/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class BallTrackingCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  int cX, cY, radius;

  /**
   * Creates a new BallTrackingCommand.
   */
  public BallTrackingCommand(DriveSubsystem driveSubsystem) {
    
    this.driveSubsystem = (DriveSubsystem) driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double yaxis = RobotContainer.getY(RobotContainer.joy1, Constants.yDeadband); // Adjusted Y

    cX = (int) SmartDashboard.getNumber("CenterX", -1.0);
    cY = (int) SmartDashboard.getNumber("CenterY", -1.0);
    radius = (int) SmartDashboard.getNumber("Radius", -1.0);

    //driveSubsystem.followBall(yaxis, cX, radius);
    if (cX != -1) {
      driveSubsystem.drive(-1 * (double) (Constants.centreScreenX - cX) * Constants.kP_balls, (double) (Constants.centreScreenX - cX) * Constants.kP_balls);
    } else driveSubsystem.drive(0, 0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.joy1.getRawButton(Constants.ballButtonNumber);
  }
}
