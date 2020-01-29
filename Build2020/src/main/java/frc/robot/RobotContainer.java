/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.PositionCommand;
import frc.robot.Commands.RotationCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.SteerCommand;
import frc.robot.Subsystems.ControlPanelSubsystem;
import frc.robot.Subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // Joystick kept public
  public static Joystick joy1 = new Joystick(1);
  public static Encoder enc_L = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
  public static Encoder enc_R = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
  public static AHRS navx = new AHRS(SPI.Port.kMXP);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem();
  
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(driveCommand);
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   
    JoystickButton commandBrakeButton = new JoystickButton(joy1, Constants.brakeButtonNumber);
    JoystickButton commandSteerButton = new JoystickButton(joy1, Constants.steerButtonNumber);
    JoystickButton rotationControlButton = new JoystickButton(joy1, Constants.rotationButtonNumber);
    JoystickButton positionControlButton = new JoystickButton(joy1, Constants.positionButtonNumber);
    
    commandBrakeButton.whenPressed(new BrakeCommand(driveSubsystem));
    commandSteerButton.whenPressed(new SteerCommand(driveSubsystem));
    rotationControlButton.whenPressed(new RotationCommand(controlPanelSubsystem));
    positionControlButton.whenPressed(new PositionCommand(controlPanelSubsystem));

  }


  public static double getY(final Joystick joy, final double band) {
    // Inverted (Joystick moved forwards gives negtive reading)
    double val = -joy.getY();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }

  public static double getZ(Joystick joy, double band) {
    double val = joy.getZ();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }

  public static double getX(Joystick joy, double band) {

    double val = joy.getX();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
