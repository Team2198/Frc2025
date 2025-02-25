// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  TalonFX driveMotor;      
  TalonFX turningMotor;
  CANcoder turningEncoder;
  double turningID;
  TalonFXConfiguration config = new TalonFXConfiguration();
  VelocityVoltage velocitySetter = new VelocityVoltage(0);
  PositionVoltage positonSetter = new PositionVoltage(0);
  double offset;
  PIDController turnPidController;
  PIDController positionController = new PIDController(2.0764, 0, 0);
  String name;

  PositionVoltage anglePositionSetter = new PositionVoltage(0);
  public SwerveModule(int driveID, int turningID, int encoderID, double offset2, boolean inverted, String name, double pid, double kv) {
    driveMotor = new TalonFX(driveID);
    turningMotor = new TalonFX(turningID);
    turningEncoder = new CANcoder(encoderID);
    //turningEncoder.setPosition(0);
    offset=offset2;
    this.name=name;
    
    this.turningID = turningID;
    
    turnPidController = new PIDController(0.005,0,0.0000);
    turnPidController.enableContinuousInput(-180, 180);
    turnPidController.setTolerance(0, 0);
    SmartDashboard.putData("pid", turnPidController);
    config.Feedback.SensorToMechanismRatio = 6.75/(Math.PI*Units.inchesToMeters(4));
    config.Slot0.kV=kv;
    config.Slot0.kP=
    config.Slot1.kP = 2.0764;
    
    if (inverted){
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    else{
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    }
    

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    encoderConfig.MagnetSensor.MagnetOffset = -offset;
    turningEncoder.getConfigurator().apply(encoderConfig);
    TalonFXConfiguration turningConfig = new TalonFXConfiguration();
    turningConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turningConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    turningConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    
    turningConfig.Slot0.kP=50;

    if (name.equals("back right")){
      turningConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    else{
      turningConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  
    } 
    config.CurrentLimits.StatorCurrentLimit = 35;
    turningConfig.CurrentLimits.StatorCurrentLimit = 40;
    turningMotor.getConfigurator().apply(turningConfig);

    driveMotor.getConfigurator().apply(config); 

    driveMotor.setPosition(0);
    SmartDashboard.putNumber("offset"+turningID, getTurningPosition()/360);

    
    
  }

  

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    
    //setAngle(0);
    SmartDashboard.putNumber("position"+name, getTurningPosition());
    SmartDashboard.putNumber("drive pos"+name, driveMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("turn coder pos"+name, turningEncoder.getPosition().getValueAsDouble());
    
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getVelocity(), new Rotation2d(Units.degreesToRadians(getTurningPosition())));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Units.degreesToRadians(getTurningPosition())));
  }

  public double getDrivePosition(){
    return driveMotor.getPosition().getValueAsDouble();
  }

  public void setSpeed(double speed, double turningSpeed){
   
    /* if (Math.abs(speed)>0.4){
      driveMotor.set(speed);
    }

    else{
      driveMotor.set(0);
    } */
    driveMotor.set(speed);
    //setAngle(90);

    /* if (Math.abs(turningSpeed)>0.1){
      turningMotor.set(turningSpeed);
    }
    else{
      turningMotor.set(0);
    } */
    
    
  }

  public double getTurningPosition() {
    StatusSignal<Angle> pos = turningEncoder.getPosition();
    double currAngle = (pos.getValueAsDouble());
    SmartDashboard.putNumber("originalval"+turningID, currAngle);
    
    //SmartDashboard.putNumber("positionoutdegrees3"+turningID, currAngle);
    currAngle = Math.signum(currAngle)*(Math.abs(currAngle)%1);
   // SmartDashboard.putNumber("positionoutdegrees2"+turningID, currAngle);
    if (currAngle<0){
      SmartDashboard.putBoolean("in"+turningID, true);
      currAngle = 1+currAngle;
    }
    
    

    
    
    SmartDashboard.putNumber("positionoutdegrees"+turningID, currAngle);
    //currAngle=currAngle*(150/7)*360;
    //currAngle = Math.signum(currAngle)*(Math.abs(currAngle)%360);
    //if (currAngle<0){
      //currAngle = currAngle+360;
    //}
    currAngle = currAngle*360;
    if (currAngle>180){
      return currAngle-360;
    }
    return currAngle;
  }

  public void setState(SwerveModuleState state){

      if (Math.abs(state.speedMetersPerSecond)<0.001){
        SmartDashboard.putNumber("velcity"+name, 0);
        SmartDashboard.putNumber("goal", 0);
        SmartDashboard.putNumber("angle"+name, 0);

        turningMotor.set(0);
        driveMotor.set(0);
        return;
      }
      else{
        SmartDashboard.putNumber("angle unoptimized"+name, Math.toDegrees(state.angle.getRadians()));
        double currAngle = getTurningPosition();
        state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(currAngle)));
        SmartDashboard.putNumber("velcity"+name, state.speedMetersPerSecond);
        SmartDashboard.putNumber("angle optimized"+name, Math.toDegrees(state.angle.getRadians()));
        SmartDashboard.putNumber("curr angle"+name, currAngle);
        SmartDashboard.putNumber("curr velocity"+name, getVelocity());
        setAngle(Math.toDegrees(state.angle.getRadians()));
        
        driveMotor.setControl(velocitySetter.withVelocity(state.speedMetersPerSecond).withSlot(0));
        //driveMotor.set(state.speedMetersPerSecond/4.89);
        SmartDashboard.putNumber("velocity voltage app"+name, driveMotor.getMotorVoltage().getValueAsDouble());

      }
      
      

    }

    public double getVelocity(){
      return driveMotor.getVelocity().getValueAsDouble();
    }




  public void setAngle(double angle){
    
    
    //SwerveModuleState state = SwerveModuleState.optimize(null, null)
    double voltageApp= turnPidController.calculate(getTurningPosition(), angle);
    //setSpeed(0,voltageApp);
    
    //SmartDashboard.putNumber("goal",angle);
    if (name.equals("back right")){
      turningMotor.set(-voltageApp);
    }
    else{
      turningMotor.set(voltageApp);
    }
    

    //Shuffleboard.getTab("Module turn").add("voltage app "+name, voltageApp);
    //Shuffleboard.getTab("Module turn").add("wheel angle "+name, getTurningPosition());
    //Shuffleboard.getTab("Module turn").add("target angle "+name, angle);
    
    //SmartDashboard.putNumber("error", turnPidController.getPositionError());

   

    //turningMotor.setControl(anglePositionSetter.withPosition(actualAngle));
    SmartDashboard.putNumber("voltage app "+name, voltageApp);
    SmartDashboard.putNumber("goal hello"+name, angle);

    
  }

  public boolean atSetpoint(){
    return turnPidController.atSetpoint();
  }

  public boolean followPath(double voltage){
    driveMotor.setVoltage(voltage);
    
    //driveMotor.setControl(positonSetter.withPosition(target).withSlot(1));
    return positionController.atSetpoint();
  }

  public void zeroDrive(){
    driveMotor.setPosition(0);
  }


}
