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

    // CAN IDs
    public final static int FR_port = 3;
    public final static int FL_port = 2;
    public final static int BR_port = 4;
    public final static int BL_port = 1;

    public final static int CP_port = 8;

    public final static double wheelMaxSpeed = 0.3;
    public final static double maxSpeed = 0.4;
    public final static double swerveCoefficient = 0.4;
    public final static double yDeadband = 0.05;
    public final static double zDeadband = 0.2;

    public final static double integralResetBound = 0.1;
    public final static double zTurnThreshold = 0.1;

    // PID Constants // Do it better
    public static double kP_DriveStraight = 4.0 * Math.pow(10, -4);
    public static double kI_DriveStraight = 1.5 * Math.pow(10, -4);
    public static double kD_DriveStraight = 3.0 * Math.pow(10, -4);

    public static double kP_DriveTurn = 0;
    public static double kI_DriveTurn = 0;
    public static double kD_DriveTurn = 0;
    

    public static double kP_NavX = 0;
    public static double kI_NavX = 0;
    public static double kD_NavX = 0;


    //Buttons
    public final static int steerButtonNumber = 2;
    public final static int brakeButtonNumber = 7;
    public final static int rotationButtonNumber = 3;
    public final static int positionButtonNumber = 4;

    //Sigmoid function
    public static final double sigmoid(double value) {
        return (1/(1 + Math.pow(Math.E, (-1*value))));
    }


}
