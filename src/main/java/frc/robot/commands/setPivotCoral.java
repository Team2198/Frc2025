// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setPivotCoral extends Command {
  /** Creates a new setPivotCoral. */
  DoubleSupplier angle;
  Coral coral;
  public setPivotCoral(Coral coralSub, DoubleSupplier angleSup) {
    coral = coralSub;
    angle = angleSup;
    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.rotateToAngle(67.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.keepUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(coral.getPivotAngle()-angle.getAsDouble())<=3;
  } 
}
