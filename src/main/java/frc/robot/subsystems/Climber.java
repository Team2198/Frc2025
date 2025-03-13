// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final SparkMax climberPivot = new SparkMax(15, MotorType.kBrushless);
  SparkMaxConfig configClimber = new SparkMaxConfig();
  private final RelativeEncoder climberEncoder  = climberPivot.getEncoder();
  public Climber() {
    configClimber.inverted(false);
    configClimber.smartCurrentLimit(30);
    configClimber.idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber voltage", climberPivot.getAppliedOutput()*12);
    SmartDashboard.putNumber("climber height", getClimber());
    // This method will be called once per scheduler run
  }

  public double getClimber(){
    
    return climberEncoder.getPosition()*(1/144);
  }


  public void setClimber(double voltage){
    climberPivot.setVoltage(voltage);
  }
}
