// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  /** Creates a new Coral. */
  
  private final SparkMax motorPivot = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax coralDropper = new SparkMax(4, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder  = motorPivot.getEncoder();
  PIDController pivotPid = new PIDController(0, 0, 0);
  SparkMaxConfig configPivot = new SparkMaxConfig();
  SparkMaxConfig coralConfig= new SparkMaxConfig();
  public Coral() {
    configPivot
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    configPivot.encoder
    //1:15 ratio
    .positionConversionFactor(1/15)
    .velocityConversionFactor(1/15);
    configPivot.smartCurrentLimit(30);
    configPivot.signals.primaryEncoderVelocityPeriodMs(100);
    motorPivot.configure(configPivot, null, null);

    coralConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    coralConfig.encoder
    //1:15 ratio
    .positionConversionFactor(1/15)
    .velocityConversionFactor(1/15);
    coralConfig.signals.primaryEncoderPositionPeriodMs(100);
    coralConfig.signals.primaryEncoderVelocityPeriodMs(100);
    coralConfig.smartCurrentLimit(30);
    coralDropper.configure(coralConfig, null, null);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivor position", getPivotAngle());
    
  }

  public void rotatePivot(double angle){
    setVoltagePivot(pivotPid.calculate(angle));
  }

  public void setVoltagePivot(double voltage){
    motorPivot.setVoltage(voltage);
  }

  public double getPivotAngle(){
    return pivotEncoder.getPosition()*360;
  }

  public void setVoltageDropper(double voltage){
    coralDropper.setVoltage(voltage); 
  }


}
