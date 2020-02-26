/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFastCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  
  /**
   * Creates a new IntakeFastCommand.
   */
  public IntakeFastCommand(Subsystem intakeSubsystem) {
    this.intakeSubsystem = new IntakeSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double speed = RobotContainer.getY(RobotContainer.joy2, Constants.yDeadband);
    intakeSubsystem.setIntakeSpeed(0.7*RobotContainer.IntakeSign);
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

