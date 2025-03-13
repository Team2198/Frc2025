// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {
  /** Creates a new CoralPivot. */
  Coral coral;
  boolean intake = true;
  public CoralIntake(Coral coralSub) {
    coral = coralSub;
    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (coral.getPivotAngleDegrees() <= -67){
      coral.setVoltageDropper(0);  
    }
    else if (coral.getPivotAngleDegrees()<=-21.5){
      intake = true;
      coral.setVoltageDropper(-0.25*12);
    }

    else{
      intake = false;
      coral.setVoltageDropper(0.35*12);
    }

    
   //coral.setVoltagePivot(0);
   //coral.applyfeedForward();
    //coral.rotateToAngle(90);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.setVoltageDropper(-0.2);
    //coral.applyfeedForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake){
      if (coral.getPivotAngleDegrees() <= -67){
        return true;
      }
      else{
        return coral.getBeamBroken();
      }
      
    }
    else{
      return false;
    }
    
  }
}
