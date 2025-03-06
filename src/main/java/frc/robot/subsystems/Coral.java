// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  /** Creates a new Coral. */
  
  private final SparkMax motorPivot = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax coralDropper = new SparkMax(2, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder  = motorPivot.getEncoder();
  PIDController pivotPid = new PIDController(0, 0, 0);
  SparkMaxConfig configPivot = new SparkMaxConfig();
  SparkMaxConfig coralConfig= new SparkMaxConfig();
  
  public Coral() {
    configPivot.inverted(false);
    configPivot.smartCurrentLimit(30);
    configPivot.idleMode(IdleMode.kBrake);
    //configPivot.encoder.positionConversionFactor(1/15);
    //configPivot.encoder.velocityConversionFactor(1/15);
    

     
    motorPivot.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /* coralConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    coralConfig.encoder
    //1:15 ratio
    .positionConversionFactor(1/15)
    .velocityConversionFactor(1/15);
    coralConfig.signals.primaryEncoderPositionPeriodMs(100);
    coralConfig.signals.primaryEncoderVelocityPeriodMs(100);
    coralConfig.smartCurrentLimit(30);
    coralDropper.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); */

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //rotatePivot(60);

    SmartDashboard.putNumber("Pivot position", getPivotAngle());
    
  }

  public void setRight(double voltage){
    motorPivot.set(voltage);
  }

  public double keepUp(){
    double angle = getPivotAngle();
    if (getPivotAngle()<=90){
      angle = 90-getPivotAngle();
    } 

    else{
      angle = getPivotAngle()-90;
    }
    
    
    if (getPivotAngle()<=25){
      setVoltagePivot(0);
      SmartDashboard.putNumber("voltage to hold up",0);
      return 0;
    }
    else{
      SmartDashboard.putNumber("voltage to hold up",Math.cos(Math.toRadians(angle))*6);
      setVoltagePivot(.5);
      return 0.5;
      
    }
    //
    //setVoltagePivot(pivotPid.calculate(angle));
  }

  public void rotateToAngle(double angle){
    double voltage;
    if (getPivotAngle()<=angle){
      voltage = -1;
    }
    else{
      voltage = 1;
    }

    setVoltagePivot(voltage);
  }

  public void setVoltagePivot(double voltage){
    motorPivot.setVoltage(voltage);
  }

  public double getPivotAngle(){
    return pivotEncoder.getPosition()*360*1/9*-1+20;
  }

  

  public void setVoltageDropper(double voltage){
    coralDropper.setVoltage(voltage);
  }


  public Command keepUpCom(){
    return this.run(()->keepUp());
  }

  public Command stopCom(){
    return this.runOnce(()->setVoltagePivot(0));
  }


}
