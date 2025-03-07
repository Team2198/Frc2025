// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.GeneralSecurityException;

import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLight extends SubsystemBase {

  private final AddressableLED m_led = new AddressableLED(0);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
  private Color currentColor; 
  
  
  /** Creates a new LEDLight. */
  public LEDLight() {
    //Initialize 
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);

  }

  public void changeColor(Color colorS){

    if (!this.currentColor.equals(colorS)){
      LEDPattern color = LEDPattern.solid(colorS);
      color.applyTo(m_ledBuffer);
      m_led.setData(m_ledBuffer);
    }
    
  }

  public void defaultColor(){
    if (!this.currentColor.equals(Color.kBlue)){
      LEDPattern color = LEDPattern.solid(Color.kBlue);
      color.applyTo(m_ledBuffer);
      m_led.setData(m_ledBuffer);
    }
  }

  public boolean getLimelightTV(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0) == 1; 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getLimelightTV()){
      changeColor(Color.kRed);
    }

    else{
      defaultColor();
    }

  }
}
