// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLights extends SubsystemBase {

  private final AddressableLED m_led = new AddressableLED(0);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
  private Color currColour; 
  private boolean hasCoral; 
  private boolean hasApril; 

  /** Creates a new LEDLights. */
  public LEDLights() {

    currColour = Color.kBlue; 
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setColor(Color color){

    if (!currColour.equals(color)){
      LEDPattern red = LEDPattern.solid(color);
      red.applyTo(m_ledBuffer);
      m_led.setData(m_ledBuffer);

      currColour = color; 
    }

  }

  public void updateHasCoral(boolean coral){
    hasCoral = coral;
  }
  
  public void updateHasApril(boolean april){
    hasApril = april;
  }

  public void setDefaultColor(){

    if (hasApril){
      setColor(Color.kGreen);
    }

    else if (hasCoral){
      setColor(Color.kRed); //Green
    }

    else{
      setColor(Color.kBlue); 
    }
  }

  @Override
  public void periodic() {
    setDefaultColor();
  }
}
