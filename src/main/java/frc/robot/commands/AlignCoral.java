// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSub;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StructPublisher;


/* You should consider using the more terse Command factories API istead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCoral extends Command {
  /** Creates a new AlignCoral. */

  private final DriveSub drive; 
  private final int pipeLine; 

  public AlignCoral(DriveSub drive, int pipeLine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive; 
    this.pipeLine = pipeLine; 

    addRequirements(this.drive);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.changeLimelightPipeLine(pipeLine);

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
    //drive.followPathNew();
    
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