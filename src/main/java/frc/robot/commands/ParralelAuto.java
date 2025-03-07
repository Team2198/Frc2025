package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSub;

public class ParralelAuto {
    DriveSub drive;
    
    public ParralelAuto(DriveSub drive){
        this.drive = drive;
        
        
        
    }

    


    public Command driveBack(double distance, double angle){
        return Commands.parallel(new AutoDrive(drive, distance, angle).withTimeout(3.5));
    }


    


   


}
