// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAprilTagHorizontal extends Command {
  /** Creates a new AlignAprilTagHorizontal. */
 private DriveSub drive; 

  public AlignAprilTagHorizontal(DriveSub drive) {
    this.drive = drive; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double data = drive.getLimelightTX(); 
    
    if (data > 0){
      data = 0.5; 
    }

    else if (data < 0){
      data = -0.5;
    }

    drive.robotRelative(0, data, 0);
    SmartDashboard.putNumber("April tag horizontal direction", data);

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
