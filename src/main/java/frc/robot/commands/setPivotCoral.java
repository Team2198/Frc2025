// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setPivotCoral extends Command {
  /** Creates a new setPivotCoral. */
  DoubleSupplier angle;
  Coral coral;
  Double initialAngle;
  public setPivotCoral(Coral coralSub, DoubleSupplier angleSup) {
    coral = coralSub;
    angle = angleSup;
    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = coral.getPivotAngle();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //coral.setVoltagePivot(0.3);
    //coral.setVoltagePivot(coral.getfeedforward());
    //coral.setVoltagePivot(coral.getfeedforward());
    
    /* SmartDashboard.putNumber("pivot target", angle.getAsDouble());
    if (angle.getAsDouble()==130||angle.getAsDouble()==67.5){
      coral.rotateToAngle(angle.getAsDouble());
      SmartDashboard.putBoolean("done", false);  
    }
    else{
      coral.rotateToAngleTwo(angle.getAsDouble());
      SmartDashboard.putBoolean("done", false);
    } 
    SmartDashboard.putNumber("initial angle", initialAngle); */
    coral.rotateToAngle(angle.getAsDouble());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //coral.applyfeedForward();
    //coral.getPivotPid().reset();
    
    //coral.setVoltagePivot(coral.getfeedforward());
    SmartDashboard.putBoolean("done", true);
    //coral.setVoltagePivot(0);
    
    
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    /* if (angle.getAsDouble()<initialAngle){
      
      SmartDashboard.putBoolean("done", false); 
      return coral.getPivotpidThree().atSetpoint(); 
    }
    else{
      if (angle.getAsDouble()<110){
        return coral.getPivotPid().atSetpoint();
      }
      
      return coral.getPivotPidTwo().atSetpoint();
      
    } */
    
    /* if (angle.getAsDouble()==130||angle.getAsDouble()==67.5){
      SmartDashboard.putBoolean("done", true);
      return coral.atSetpoint();
        
    }
    else{
      SmartDashboard.putBoolean("done", true);
      return false;
      //return coral.rotateToAngleTwo(25);
      
    }  */

    return false;
   
    
    
  } 
}
