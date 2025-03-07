// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
  /** Creates a new Algae. */

  private final SparkMax motorLeft = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax motorRight = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax motorPivot = new SparkMax(4, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder  = motorPivot.getEncoder();
  private boolean beamBroken = false;
  PIDController pivotPid = new PIDController(0, 0, 0);
  SparkMaxConfig configPivot = new SparkMaxConfig();
  SparkMaxConfig configRight = new SparkMaxConfig();
  SparkMaxConfig confightLeft = new SparkMaxConfig();
  //DigitalInput beamBreak = new DigitalInput(0);


  public Algae() {

    //config algae pivot 
    configPivot
    .inverted(false)
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(30);
    motorPivot.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    

    configRight
    .inverted(false)
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(30);
    motorRight.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    configRight
    .inverted(false)
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(30);
    motorLeft.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    /* configPivot
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    configPivot.encoder
    .positionConversionFactor(1/48);
    //.velocityConversionFactor(1/48);
    configPivot.smartCurrentLimit(30);
    configPivot.signals.primaryEncoderVelocityPeriodMs(100);
    
    motorPivot.configure(configPivot, null, null);

    //config left and right pivot motors
    
    configRight
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    configPivot.encoder
    .positionConversionFactor(1/20)
    .velocityConversionFactor(1/20);
    configPivot.smartCurrentLimit(30);
    configPivot.signals.primaryEncoderVelocityPeriodMs(100);

    confightLeft
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    configPivot.encoder
    .positionConversionFactor(1/20)
    .velocityConversionFactor(1/20);
    configPivot.smartCurrentLimit(30);
    configPivot.signals.primaryEncoderVelocityPeriodMs(100);
    
    motorRight.configure(configRight, null, null);
    motorRight.configure(configRight, null, null); */
  }

  public double getfeedforward(){
    double angle = getPivotAngle();
    if (angle<=90){
      angle = 90-getPivotAngle();
    } 

    else{
      angle = angle-90;
    }
    double voltage = Math.cos(Math.toRadians(angle)*0.5);
    
    return voltage;
    
      
    
  }

  public void rotateToAngle(double angle){
    double voltage = pivotPid.calculate(getPivotAngle(), angle);
    SmartDashboard.putNumber("arm feedforward", getfeedforward());
    SmartDashboard.putNumber("voltage applied rotation", voltage);
    

    voltage+=getfeedforward();
    motorPivot.setVoltage(voltage);
    
    
  }

  public void applyfeedForward(){
    if (getPivotAngle()>=15){
      motorPivot.set(getfeedforward());
    }
    else{
      motorPivot.set(0);
    }
    

  }

  public void setVoltagePivot(double voltage){
    motorPivot.setVoltage(voltage);
  }

  public double getPivotAngle(){
    return pivotEncoder.getPosition()*360;
  }

  public void setVoltageDropper(double voltage){
    motorLeft.setVoltage(voltage);
    motorRight.setVoltage(voltage);
  }

  /* public void checkBeamStatus(){
    if (true){    
      beamBroken = true;
      SmartDashboard.putBoolean("Algae Last Detected", beamBroken);
    }
    else{
      if (beamBroken){
        beamBroken=false;
        SmartDashboard.putBoolean("Algae Last Detected", beamBroken);
      }
      
    }
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //checkBeamStatus();
  }


  public void setRight(double voltage){
    motorRight.setVoltage(voltage);
    
    

  }

  public void setLeft(double voltage){
    motorLeft.setVoltage(voltage);
    
    
    
  }
}
