// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSub;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAprilTag extends Command {
  /** Creates a new AlignAprilTagHorizontal. */
 private DriveSub drive; 

  public AlignAprilTag(DriveSub drive) {
    this.drive = drive; 
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (this.drive.getLimelightTV()){
      double[] botPose = this.drive.getLimelightBotpose();
      double x = botPose[0];
      double y = botPose[1];
      Rotation2d yaw = Rotation2d.fromDegrees(botPose[5]); //you will have to test whether or not you need to negative it 

      SmartDashboard.putString("limelight yaw", Arrays.toString(botPose));
      
      drive.followPathNew(new Pose2d(x, y, yaw));
    }
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
