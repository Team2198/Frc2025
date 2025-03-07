// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.setPivotCoral;

public class Coral extends SubsystemBase {
  /** Creates a new Coral. */
  
  private final SparkMax motorPivot = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax coralDropper = new SparkMax(2, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder  = motorPivot.getEncoder();
  PIDController pivotPid = new PIDController(0.02, 0, 0);
  PIDController pivotPidTwo = new PIDController(0.15  ,0,0);
  PIDController pivotPidThree = new PIDController(0.05, 0,0);
  SparkMaxConfig configPivot = new SparkMaxConfig();
  SparkMaxConfig coralConfig= new SparkMaxConfig();
  SparkClosedLoopController pidController;
  double factor = 1.0/9.0;
  public Coral() {
    configPivot.inverted(false);
    configPivot.smartCurrentLimit(30);
    configPivot.idleMode(IdleMode.kBrake);
    
    
    configPivot.encoder
      .positionConversionFactor(factor)
      .velocityConversionFactor(factor);
    
    configPivot.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0,0,0);

    

    
    pivotPid.setTolerance(5);
    pivotPidTwo.setTolerance(5);
    pivotPidThree.setTolerance(5);
    
    pidController = motorPivot.getClosedLoopController();
    


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

  public double getfeedforward(double angle){
    /* double angle = getPivotAngle();
    if (angle<=127){
      angle = 127-getPivotAngle();
      if (angle>=90){
        angle = 90;
      }
    } 

    else{
      angle = angle-127;
    }
    double voltage = Math.cos(Math.toRadians(angle))*0.6;//0.7

     if (getPivotAngle()>=110){
      voltage = 0.7;  
    } */ 
    double voltage = -pivotPidTwo.calculate(getPivotAngle(), angle);
    SmartDashboard.putNumber("feedforward angle", angle);
    SmartDashboard.putNumber("applied feedforward volts", voltage);
    return voltage;
    
      
    
  }

  public double getfeedforward(){
     double angle = getPivotAngle();
    if (angle<=127){
      angle = 127-getPivotAngle();
      if (angle>=90){
        angle = 90;
      }
    } 

    else{
      angle = angle-127;
    }
    double voltage = Math.cos(Math.toRadians(angle))*0.6;//0.7

     if (getPivotAngle()>=110){
      voltage = 0.7;  
    } 
    
    SmartDashboard.putNumber("feedforward angle", angle);
    SmartDashboard.putNumber("applied feedforward volts", voltage);
    return voltage;
    
      
    
  }

  public void rotateToAngle(double angle){
    double voltage = 0; 

    if (angle - getPivotAngle() > 0){
      voltage = -pivotPid.calculate(getPivotAngle(), angle);

      SmartDashboard.putBoolean("Up?", false);
    }

    else {
      voltage = getfeedforward();
      SmartDashboard.putBoolean("Up?", true);
    }

    
    
    /* if (angle>=100){
      voltage = -pivotPidTwo.calculate(getPivotAngle(),angle);
    } */
    //SmartDashboard.putNumber("arm feedforward", getfeedforward());
    SmartDashboard.putNumber("voltage applied rotation", voltage);
    

    //voltage+=getfeedforward(angle);
    //SmartDashboard.putNumber("voltage applied rotation 2", voltage);
    //voltage = voltage+0.225*Math.signum(voltage);

    setVoltagePivot(voltage);

    
  }

  public boolean rotateToAngleTwo(double angle){
    double voltage = 0;
    // voltage = -pivotPidThree.calculate(getPivotAngle(), angle);
    
    /* if (angle>=100){ */
    //voltage = -pivotPidThree.calculate(getPivotAngle(),angle);
    /* } */
    SmartDashboard.putNumber("arm feedforward", getfeedforward());
    SmartDashboard.putNumber("voltage applied rotation", voltage);
    

    voltage = 1.7;

    if (getPivotAngle()>=30){
      setVoltagePivot(voltage);
      return false;
    }
    else{
      setVoltagePivot(0);
      return true;
    }
    
       
    
  }

  public void applyfeedForward(){
    if (getPivotAngle()>=30){

      if (getPivotAngle()>=50){
        setVoltagePivot(getfeedforward());
        SmartDashboard.putNumber("applied feedforward", getfeedforward());
      }
      else{
        setVoltagePivot(0.1);
      }
      
      
    }
    else{
      //motorPivot.set(0);
      setVoltagePivot(0);
      SmartDashboard.putNumber("applied feedforward", 0);
    }
    

  }

  public PIDController getPivotPid(){
    return pivotPid;
  }

  public boolean atSetpoint(){
    SmartDashboard.putBoolean("at setpoint", pivotPid.atSetpoint());
    return pivotPid.atSetpoint();
  }

  public void setVoltagePivot(double voltage){
    SmartDashboard.putNumber("voltage pivot", voltage);
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

  public PIDController getPivotPidTwo() {
    // TODO Auto-generated method stub
    return pivotPidTwo;
  }

  


}
