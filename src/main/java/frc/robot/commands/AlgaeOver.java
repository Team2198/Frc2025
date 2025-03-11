// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeOver extends Command {
  /** Creates a new AlgeaOver. */
  Algae algea;
  double degrees;
  int forward; 

  public AlgaeOver(Algae algeaSub, double degrees, int forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    algea = algeaSub;
    addRequirements(algeaSub);
    this.degrees = degrees; 
    this.forward = forward; 
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // algea.setRight(2);
    //algea.setVoltageDropper(speedLeft*12);

      algea.rotateToAngle(degrees);
      
      if (forward == 1){
        algea.setLeft(5);
        algea.setRight(5);
      }

      else if (forward == 2){
        algea.setLeft(-5);
        algea.setRight(-5);
      }
      
      else if (forward == 3){
        algea.setLeft(0);
        algea.setRight(0);
      }

      else if (forward == 4){
        algea.setLeft(-12);
        algea.setRight(-12);
      }

      else{
        algea.setLeft(3);
        algea.setRight(3);
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
