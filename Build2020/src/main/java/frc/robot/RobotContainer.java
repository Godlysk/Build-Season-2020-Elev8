/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlPanelCommands.PositionCommand;
import frc.robot.ControlPanelCommands.RotationCommand;
import frc.robot.DriveCommands.SteerCommand;
import frc.robot.SolenoidCommands.DownCommand;
import frc.robot.SolenoidCommands.UpCommand;
import frc.robot.Subsystems.AutoDriveSubsystem;
import frc.robot.Subsystems.ControlPanelSubsystem;
import frc.robot.Subsystems.DifferentialDriveSubsystem;
import frc.robot.Subsystems.SolenoidSubsystem;

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
  public static AutoDriveSubsystem m_robotDrive = new AutoDriveSubsystem();


  public final DifferentialDriveSubsystem driveSubsystem2 = new DifferentialDriveSubsystem(navx);
  public final ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem();
  public final SolenoidSubsystem solenoidSubsystem = new SolenoidSubsystem();
  
  public final SteerCommand steerCommand = new SteerCommand(driveSubsystem2);
  public final UpCommand upCommand = new UpCommand(solenoidSubsystem);

  

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem2.setDefaultCommand(steerCommand);
    solenoidSubsystem.setDefaultCommand(upCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   
    //final JoystickButton commandBrakeButton = new JoystickButton(joy1, Constants.brakeButtonNumber);
    // JoystickButton commandSteerButton = new JoystickButton(joy1,
    // Constants.steerButtonNumber);
    final JoystickButton rotationControlButton = new JoystickButton(joy1, Constants.rotationButtonNumber);
    final JoystickButton positionControlButton = new JoystickButton(joy1, Constants.positionButtonNumber);
    final JoystickButton solenoidButton = new JoystickButton(joy1, Constants.solenoidButtonNumber);

    // commandBrakeButton.whenPressed(new BrakeCommand(driveSubsystem));
    // commandSteerButton.whenPressed(new SteerCommand(driveSubsystem));

    // rotationControlButton.whenPressed(new
    // RotationCommand(controlPanelSubsystem));
    // positionControlButton.whenPressed(new
    // PositionCommand(controlPanelSubsystem));

    rotationControlButton.toggleWhenPressed(new RotationCommand(controlPanelSubsystem));
    positionControlButton.toggleWhenPressed(new PositionCommand(controlPanelSubsystem));

    solenoidButton.toggleWhenPressed(new DownCommand(solenoidSubsystem));

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

  public static double getZ(final Joystick joy, final double band) {
    double val = joy.getZ();

    if (Math.abs(val) < band)
      val = 0;
    else {
      val = val - Math.signum(val) * band;
    }
    return val;
  }

  public static double getX(final Joystick joy, final double band) {

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

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
new SimpleMotorFeedforward(Constants.kS,Constants.kV,Constants.kA),Constants.kDriveKinematics,10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.kS,Constants.kV,Constants.kA),
        Constants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}