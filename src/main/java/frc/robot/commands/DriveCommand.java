// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSub;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  
  DriveSub drive;
  DoubleSupplier xSpeed;
  DoubleSupplier ySpeed;
  SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  SlewRateLimiter turningLimiter = new SlewRateLimiter(3);
  DoubleSupplier turningSpeed;
  BooleanSupplier robotRelativeDrive;
  BooleanSupplier robotRelative;
  BooleanSupplier robotAlignApril;
  BooleanSupplier robotAlignAprilAngle;  
  public DriveCommand(DriveSub driveS, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier turningSpeed, BooleanSupplier robotRelative, BooleanSupplier robotRelativeDrive, BooleanSupplier robotAlignApril, BooleanSupplier robotAlignAprilAngle) {
    drive = driveS;
    this.robotRelative = robotRelative;
    this.xSpeed=xSpeed;
    this.ySpeed=ySpeed;
    this.turningSpeed = turningSpeed;
    this.robotRelativeDrive = robotRelativeDrive;
    this.robotAlignApril = robotAlignApril; 
    this.robotAlignAprilAngle = robotAlignAprilAngle; 
    
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double limelightData = -drive.getLimelightAlgae();
    
    double robotTurnGoal =  drive.getHeading() + limelightData;

    double maxTurningRad = Constants.TeleOp.maxTurningRad;
    double xSpeedDub = xSpeed.getAsDouble();


    

    //y joystick is inverted
    xSpeedDub = -xSpeedDub;
    double ySpeedDub = ySpeed.getAsDouble();
    ySpeedDub=-ySpeedDub;


    if (Math.abs(xSpeedDub)<0.2){
      xSpeedDub = 0;
    }

    if (Math.abs(ySpeedDub)<0.2){
      ySpeedDub = 0;
    }


        
    xSpeedDub = xLimiter.calculate(xSpeedDub);
    ySpeedDub = yLimiter.calculate(ySpeedDub);

    
    
    //StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    //.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    //drive.setSpeed(driveSpeed.getAsDouble(), turnSpeed.getAsDouble());
    double turningSpeedDub= -turningSpeed.getAsDouble();
    
    if (Math.abs(turningSpeedDub)<0.2){
      turningSpeedDub=0;
    }

    turningSpeedDub = turningLimiter.calculate(turningSpeedDub);
    turningSpeedDub *= maxTurningRad;
    
    xSpeedDub*=Constants.TeleOp.maxSpeed;
    ySpeedDub*=Constants.TeleOp.maxSpeed;
    SmartDashboard.putNumber("max turning rad", maxTurningRad);

    if (Math.abs(turningSpeedDub)<0.2){
      turningSpeedDub = 0;
    }

    

    if(Math.abs(ySpeedDub)<0.15){
      ySpeedDub=0;
    }
    /* turningSpeedDub*=0.6;
    
    xSpeedDub*=0.5;
    ySpeedDub*=0.5;
    turningSpeedDub*=0.5; */
    turningSpeedDub*=0.7;
    
    if (robotRelativeDrive.getAsBoolean()){
      drive.robotRelative(xSpeedDub, ySpeedDub, turningSpeedDub);
      SmartDashboard.putString("drive type", "robot relative");
    }
    
    else if (robotRelative.getAsBoolean()){
      drive.robotRelative(xSpeedDub, 0, drive.turnToAngle(robotTurnGoal));
      SmartDashboard.putString("drive type", "robot relative auto align");
    
    }

    else if (robotAlignApril.getAsBoolean()){
      drive.changeLimelightPipeLine(3);
      drive.updateRobotGoal();
      drive.followPathNew();
    }
    
    else if (robotAlignAprilAngle.getAsBoolean()){
      drive.changeLimelightPipeLine(3);
      drive.updateRobotGoalAngle();
      drive.followPathNew();
    }
    

    else{
      //drive.robotRelative(xSpeedDub, 0, drive.turnToAngle(robotTurnGoal));
      //drive.robotRelative(xSpeedDub, ySpeedDub, turningSpeedDub);
      drive.fieldRelative(xSpeedDub, ySpeedDub, turningSpeedDub);
      SmartDashboard.putString("drive type", "field relative");
    }
    
    if (!robotAlignApril.getAsBoolean() && !robotAlignAprilAngle.getAsBoolean()){
      drive.resetGoal();
    }

    
    //drive.robotRelative(xSpeedDub, ySpeedDub, 0);
    SmartDashboard.putNumber("turningSpeed", turningSpeedDub);
    SmartDashboard.putNumber("x speed command", xSpeedDub);
    SmartDashboard.putNumber("y speed command", ySpeedDub);
    //drive.setAngle(90);
    //drive.setSpeed(xSpeedDub/4.89, 0);
   // drive.setAngle(90);

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
