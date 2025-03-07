// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgeaOver;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberOver;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOver;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
//import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorOver;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ParralelAuto;
import frc.robot.commands.setPivotCoral;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  ParralelAuto autoSegment;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);
  
  private final CommandXboxController op_drivController =
      new CommandXboxController(1);


    DriveSub drive = new DriveSub();
    Elevator elevator = new Elevator();  
    //Algae algae = new Algae();
    Coral coral = new Coral();
    Climber climber = new Climber();
    Command driveBack;
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoSegment = new ParralelAuto(drive);
    driveBack = Commands.sequence(drive.setWheelAngleCommand(0).withTimeout(1),drive.setRobotOffset(0),drive.zeroEncodersCommand(), autoSegment.driveBack(-1.4, 0));
    // Configure the trigger bindings
    m_chooser.addOption("drive back",driveBack);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //.onTrue(new ExampleCommand(m_exampleSubsystem));

    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    drive.setDefaultCommand(new DriveCommand(drive,()->m_driverController.getLeftY(), ()->m_driverController.getLeftX(), ()->m_driverController.getRightX(), ()->m_driverController.getHID().getAButton(), ()->op_drivController.getHID().getBButton()));
    //debugging commands
    //elevator.setDefaultComma  nd(new ElevatorOver(elevator, ()->op_drivController.getRightY(), ()->op_drivController.getLeftY()));
    //evator.setDefaultCommand(new ElevatorOver(elevator, ()->m_driverController.getRightY(), ()->m_driverController.getLeftY()));
    //m_driverController.a().whileTrue(elevator.runSysIdRoutine());
    //31+7/8= l2
    //
    /* m_driverController.a().onTrue(new ElevatorCommand(elevator, ()->20));
    m_driverController.b().onTrue(new ElevatorCommand(elevator, ()->30));
    m_driverController.x().onTrue(new ElevatorCommand(elevator, ()->35));
    m_driverController.y().onTrue(new ElevatorCommand(elevator, ()->10)); */
    //op_drivController.y().onTrue(new ElevatorCommand(elevator, ()->40));       
    //op_drivController.a().onTrue(new ElevatorCommand(elevator, ()->10));
    //algae.setDefaultCommand(new AlgeaOver(algae, ()->m_driverController.getRightY(), ()->m_driverController.getLeftY()));
    //coral.setDefaultCommand(new CoralOver(coral, ()->m_driverController.getRightY(), ()->m_driverController.getLeftY()));
    //op_drivController.a().onTrue(new setPivotCoral(coral, ()->67.5));
    //op_drivController.x().onTrue(new setPivotCoral(coral, ()->130));
    //coral.setDefaultCommand(new CoralOver(coral, ()->op_drivController.getLeftY(), ()->m_driverController.y().getAsBoolean())); 
    
    //climber.setDefaultCommand(new ClimberOver(climber, ()->m_driverController.getRightTriggerAxis()));
    //m_driverController.b().onTrue(new setPivotCoral(coral, ()->130));
    //m_driverController.x().onTrue(new setPivotCoral(coral, ()->30));
    //m_driverController.y().whileTrue(new CoralIntake(coral));
    //elevator.setDefaultCommand(new ElevatorOver(elevator,()->op_drivController.getLeftY()));
    //test on the fly auto
    //m_driverController.a().onTrue(AutoBuilder.followPath(drive.generatePathToReef()));
    //m_driverController.a().whileTrue(coral.keepUpCom());
    //m_driverController.a().whileFalse(coral.stopCom());             
  }                       

  /**      
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *     s
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveBack;
  }
}
