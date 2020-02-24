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
public class PIDDriveCommand extends PIDCommand {
  
  /**
   * Creates a new PIDDriveCommand.
   */
  public PIDDriveCommand(DriveSubsystem driveSubsystem) {
    
    super(

        // The PID Controller
        new PIDController(Constants.kP_tank, Constants.kI_tank, Constants.kD_tank),
        
        // This should return the measurement
        () ->  (RobotContainer.enc_L.getRate() - RobotContainer.enc_R.getRate()),
        
        // This should return the setpoint (can also be a constant)
        () -> Constants.setpoint,
        

        output -> {

          // Use the output here
          double yaxis = RobotContainer.getY(RobotContainer.joy1, Constants.yDeadband); // Adjusted Y
          double zaxis = RobotContainer.getZ(RobotContainer.joy1, Constants.zDeadband); // Adjusted Z

          SmartDashboard.putNumber("Y-AXIS", yaxis);
          SmartDashboard.putNumber("Z-AXIS", zaxis);
          SmartDashboard.putNumber("Error", (RobotContainer.enc_L.getRate() - RobotContainer.enc_R.getRate()));
          SmartDashboard.putNumber("PID output", output);
          SmartDashboard.putNumber("Sign", Math.signum((RobotContainer.enc_L.getRate() - RobotContainer.enc_R.getRate()) / output));

          if (Math.abs(zaxis) > Constants.turnThresh) driveSubsystem.tankFeedForward(zaxis * Constants.maxVelocity, -zaxis * Constants.maxVelocity, 0.0);
          else driveSubsystem.tankFeedForward(yaxis * Constants.maxVelocity, yaxis * Constants.maxVelocity, output);

        },

        // Drive Subsystem Input as a Requirement
        driveSubsystem
        
        );

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
