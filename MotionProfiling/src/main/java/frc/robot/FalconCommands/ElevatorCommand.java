/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FalconCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FalconSubsystem;
import frc.robot.Constants;

public class ElevatorCommand extends CommandBase {

  private final FalconSubsystem falconSubsystem;
  
  /**
   * Creates a new ElevatorCommand.
   */
  public ElevatorCommand(Subsystem falconSubsystem) {
    this.falconSubsystem = (FalconSubsystem) falconSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(falconSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.getY(RobotContainer.joy2, Constants.yDeadband);
    falconSubsystem.setSpeed1(speed*0.9);
    SmartDashboard.putNumber("FalconSpeed1", -speed);
    SmartDashboard.putNumber("FlaconSpeed2", speed);

    // falconSubsystem.setSpeed1(0.4*RobotContainer.directionf_1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}