/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ControlPanelCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.ControlPanelSubsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NumberedTurnCommand extends CommandBase {
  
  private final ControlPanelSubsystem controlPanelSubsystem;
  private final double turns;
  
  /**
   * Creates a new NumberedTurnCommand.
   */
  public NumberedTurnCommand(Subsystem controlPanelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controlPanelSubsystem = (ControlPanelSubsystem)controlPanelSubsystem;
    addRequirements(controlPanelSubsystem);
    turns = 2.0;//SmartDashboard.getNumber("turns", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(turns != turnsMade())
    {
      controlPanelSubsystem.setSpeed(Constants.wheelMaxSpeed);
      String color = controlPanelSubsystem.color;
      if(!controlPanelSubsystem.p_color.equals(color) && !color.equals("error")){
        controlPanelSubsystem.p_color = color;
        Robot.colors.add(controlPanelSubsystem.p_color);
      }
    }
    controlPanelSubsystem.setSpeed(0.0d);
  }
  
  public int degreesTurned(){
    return (45*(Robot.colors.size()-1));
  }

  public double turnsMade(){
    return (Robot.colors.size()-1.0)/8.0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.colors.removeAll(Robot.colors);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

