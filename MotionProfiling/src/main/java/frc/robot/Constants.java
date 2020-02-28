/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static double yDeadband = 0.05;
    public final static double zDeadband = 0.05;
    public final static double maxSpeed = 0.8;

    public final static double steerCoefficient = 0.4;
    public final static double turnThresh = 0.2;
    public final static double integralResetBound = 0.1;

    public final static int centreScreenX = 240;
    public final static int centreScreenY = 135;

    public final static int SF_port = 1;
    public final static int SR_port = 0;

    // Buttons - Joystick 1
    public final static int brakeButtonNumber = 7;
    public final static int ballButtonNumber = 2;
    public final static int solenoidButtonNumber = 5;

    // CAN IDs
    public final static int FR_port = 3;
    public final static int FL_port = 2;
    public final static int BR_port = 4;
    public final static int BL_port = 1;

    public final static double kP_tank = 0.0;
    public final static double kI_tank = 0.0;
    public final static double kD_tank = 0.0;

    public final static double kP_steer = 0.0;
    public final static double kI_steer = 0.0;
    public final static double kD_steer = 0.0;

    public static double kP_custom = 0.0;
    public static double kI_custom = 0.0;
    public static double kD_custom = 0.0;

    public static double kP_balls = 0.00075;

    public final static double kS = 0.3;
    public final static double kV = 1.0;
    public final static double kA = 0.0;

    // PID
    public final static double setpoint = 0.0;
    public final static double maxVelocity = 4.0;


    public final static double maxIntakeSpeed = 0.4;
    public final static double maxShooterSpeed = 0.9;

    public final static int intakeMasterPort = 1;
    public final static int intakeSlavePort = 2;
    public final static int shooterPort = 3; 

    public final static int falcon1Port = 6; 
    public final static int falcon2Port = 5; 


}
