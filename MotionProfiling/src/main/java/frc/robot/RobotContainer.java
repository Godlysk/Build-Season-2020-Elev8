/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.FalconCommands.FastCommand1;
import frc.robot.FalconCommands.FastCommand2;
import frc.robot.FalconCommands.SlowCommand1;
import frc.robot.FalconCommands.SlowCommand2;
import frc.robot.FalconCommands.SwitchSign1;
import frc.robot.FalconCommands.SwitchSign2;
import frc.robot.ShooterCommands.ShooterSubCommand;
import frc.robot.SolenoidCommands.DownCommand;
import frc.robot.SolenoidCommands.UpCommand;
// import frc.robot.commands.ArcadeDriveCommand;
// import frc.robot.commands.BallTrackingCommand;
// import frc.robot.commands.CustomPIDDrive;
// import frc.robot.commands.PIDDriveCommand;
// import frc.robot.commands.StopMotorsCommand;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FalconSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  public static Joystick joy1 = new Joystick(1);
  public static Joystick joy2 = new Joystick(2);
  public static Encoder enc_L = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
  public static Encoder enc_R = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  public static AHRS navx = new AHRS(SPI.Port.kMXP);
  public static double directionf_1 = 1.0; 
  public static double directionf_2 = 1.0;

  // private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  // private final ArcadeDriveCommand arcadeDriveCommand = new ArcadeDriveCommand(driveSubsystem);
  // private final CustomPIDDrive customDriveCommand = new CustomPIDDrive(driveSubsystem);
  // private final PIDDriveCommand pidTankDriveCommand = new PIDDriveCommand(driveSubsystem);

  private final SolenoidSubsystem solenoidSubsystem = new SolenoidSubsystem();
  private final UpCommand upCommand = new UpCommand(solenoidSubsystem);
  private final FalconSubsystem falconSubsystem = new FalconSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShooterSubCommand shooterSubCommand = new ShooterSubCommand(shooterSubsystem);
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // driveSubsystem.setDefaultCommand(arcadeDriveCommand);
    solenoidSubsystem.setDefaultCommand(upCommand);
    shooterSubsystem.setDefaultCommand(shooterSubCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // JoystickButton commandBrakeButton = new JoystickButton(joy1, Constants.brakeButtonNumber);
    // // commandBrakeButton.whenPressed(new StopMotorsCommand(driveSubsystem));

    // JoystickButton ballTrackingButton = new JoystickButton(joy1, Constants.ballButtonNumber);
    // ballTrackingButton.whenPressed(new BallTrackingCommand(driveSubsystem));

    JoystickButton solenoidButton = new JoystickButton(joy1, Constants.solenoidButtonNumber);
    solenoidButton.toggleWhenPressed(new DownCommand(solenoidSubsystem)); 


    JoystickButton falcon1FastButton = new JoystickButton(joy1, 8);
    falcon1FastButton.toggleWhenPressed(new FastCommand1(falconSubsystem));

    JoystickButton falcon1SlowButton = new JoystickButton(joy1, 10);
    falcon1SlowButton.toggleWhenPressed(new SlowCommand1(falconSubsystem));

    JoystickButton falcon1Switch = new JoystickButton(joy1, 12);
    falcon1Switch.toggleWhenPressed(new SwitchSign1());


    JoystickButton falcon2FastButton = new JoystickButton(joy1, 7);
    falcon2FastButton.toggleWhenPressed(new FastCommand2(falconSubsystem));

    JoystickButton falcon2SlowButton = new JoystickButton(joy1, 9);
    falcon2SlowButton.toggleWhenPressed(new SlowCommand2(falconSubsystem));

    JoystickButton falcon2Switch = new JoystickButton(joy1, 11);
    falcon2Switch.whenPressed(new SwitchSign2());
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
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

  
}
