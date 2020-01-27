/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ControlPanelCommands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Subsystems.ControlPanelSubsystem;

public class TurnToColorCommand extends CommandBase{

    private final ControlPanelSubsystem controlPanelSubsystem;
    private final String targetColor;

    public TurnToColorCommand(Subsystem controlPanelSubsystem){
        this.controlPanelSubsystem = (ControlPanelSubsystem) controlPanelSubsystem;
        addRequirements(controlPanelSubsystem);
        targetColor = "green";//SmartDashboard.getString("target_color", "");
    }

    @Override
    public void initialize() {
        controlPanelSubsystem.setSpeed(Constants.wheelMaxSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        controlPanelSubsystem.setSpeed(0.0d);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return targetColor.equals(controlPanelSubsystem.color);
    }

}
