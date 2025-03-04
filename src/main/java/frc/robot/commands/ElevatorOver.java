// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorOver extends Command {
  /** Creates a new ElevatorOver. */
  Elevator elevatorSub;
  double speedRight;
  double speedLeft;
  public ElevatorOver(Elevator elevator, DoubleSupplier SpeedRight, DoubleSupplier SpeedLeft) {
    speedRight = SpeedRight.getAsDouble();
    speedLeft = SpeedLeft.getAsDouble();
    elevatorSub = elevator;
    addRequirements(elevatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ElevatorOver(Elevator elevator, DoubleSupplier SpeedOverall) {
    speedRight = SpeedOverall.getAsDouble();
    speedLeft = SpeedOverall.getAsDouble();
    elevatorSub = elevator;
    addRequirements(elevatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSub.setRight(speedRight);
    elevatorSub.setLeft(speedLeft);
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
