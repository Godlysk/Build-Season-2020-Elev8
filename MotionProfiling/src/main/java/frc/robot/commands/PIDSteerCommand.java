/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PIDSteerCommand extends PIDCommand {
  /**
   * Creates a new PIDSteerCommand.
   */
  public PIDSteerCommand(DriveSubsystem driveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kP_steer, Constants.kI_steer, Constants.kD_steer),

        // This should return the measurement
        () -> RobotContainer.navx.getRate(),

        // This should return the setpoint (can also be a constant)
        () -> RobotContainer.getZ(RobotContainer.joy1, Constants.zDeadband) * Constants.steerCoefficient,

        // This uses the output
        output -> {
          
          // Use the output here
          double yaxis = RobotContainer.getY(RobotContainer.joy1, Constants.yDeadband); // Adjusted Y
          
          SmartDashboard.putNumber("Y-AXIS", yaxis);
          SmartDashboard.putNumber("Turn Rate", RobotContainer.navx.getRate());

          driveSubsystem.tankFeedForward(yaxis * Constants.maxVelocity, yaxis * Constants.maxVelocity, 0.0);

        },

        driveSubsystem
        
        
        );

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
