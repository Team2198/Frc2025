// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSub;

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

    if (drive.getLimelightTV()){
      double[] botPose = drive.getLimelightBotpose(); 
      double heading = drive.getHeading(); 

      SmartDashboard.putNumber("limelight heading", heading);

      double x = -botPose[0];
      double z = botPose[2]; 

      SmartDashboard.putNumber("limelight x", z);
      SmartDashboard.putNumber("limelight y", x);

      double xAway = x * Math.sin(Math.toRadians(heading)) + z * Math.cos(Math.toRadians(heading));
      double yAway = x * Math.cos(Math.toRadians(heading)) - z * Math.sin(Math.toRadians(heading));

      SmartDashboard.putNumber("xAway", xAway);
      SmartDashboard.putNumber("yAway", yAway);

      drive.updateGoals(xAway, yAway);
    }

    
    drive.followPathGoal();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
