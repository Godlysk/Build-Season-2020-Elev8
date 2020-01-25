/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.VictorSP;

public class ControlPanelSubsystem extends SubsystemBase{

    private final VictorSP CP;

    public String p_color = "";

    public double red = 0.0d;
    public double green = 0.0d;
    public double blue = 0.0d;

    public String color = "";

    private I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public ControlPanelSubsystem(){
        CP = new VictorSP(Constants.CP_port);
    }

    @Override
    public void periodic(){
        // This method will be called once per scheduler run
        RawColor detectedColor = colorSensor.getRawColor();
        double IR = colorSensor.getIR();

        red = detectedColor.red;
        green = detectedColor.green;
        blue = detectedColor.blue;
        double redgreen = red/green;
        double greenblue = green/blue;
        double redblue = red/blue;

        color = colorDetection(red, green, blue);
            
        SmartDashboard.putNumber("Red", red);
        SmartDashboard.putNumber("Green", blue);
        SmartDashboard.putNumber("Blue", green);
        SmartDashboard.putNumber("Red by Green", redgreen);
        SmartDashboard.putNumber("Green by Blue", greenblue);
        SmartDashboard.putNumber("Red by Blue", redblue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putString("Color", color);
    }

    public void setSpeed(double speed){
        CP.set(speed);
    }

    public void stopWheel(){
        CP.set(0.0);
    }

    public String colorDetection(double red, double green, double blue){
        double redgreen = red/green;
        double redblue = red/blue;
        double greenblue =green/blue;
        if(redgreen>=0.5 && redgreen<=0.65 && greenblue>=4.1 && greenblue<=5.6){
            return "yellow";
        }
        else if(redblue>=0.25 && redblue<=0.7 && greenblue>=1 && greenblue<=1.6){
            return "blue";
        }
        else if(redblue>=1 && redblue<=1.2 && redgreen>=0.275 && redgreen<=0.5){
            return "green";
        }
        else if(redblue>=1.5 && redblue<=4.9 && redgreen>=0.75 && redgreen<=1.6){
            return "red";
        }
        else{
            return "error";
        }
      }  

}