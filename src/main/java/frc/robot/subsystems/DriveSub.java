// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;


import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.MutableMeasure;

import edu.wpi.first.units.VelocityUnit;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Measure;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import frc.robot.LimelightHelpers;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.StructArrayPublisher;

public class DriveSub extends SubsystemBase {
  /** Creates a new DriveSub. */
  private final SwerveModule frontLeft = new SwerveModule(10, 13, 12,0.376709, true,"front left", 0.015, 2.4585);
  private final SwerveModule frontRight = new SwerveModule(6, 9, 8,0.323242, true,"front right",0.015, 2.4691);
  //AHRS gyro = new AHRS(null);
  PIDController positionPidController = new PIDController(4,0,0);
  int counter=0;
  double dimension = Units.inchesToMeters(27/2);
  double dimensionTwo = Units.inchesToMeters(32/2);
  //private final SwerveModule backLeft = new SwerveModule(20, 21, 26, 0.268311, true, "back left", 0.015, 2.4978);
  private final SwerveModule backLeft = new SwerveModule(14, 17, 16, 0.261963, true, "back left", 0.015, 2.4978);
  
  //private final SwerveModule backRight = new SwerveModule(14, 15, 22,0.133301, true, "back right", 0.015, 2.46);
  private final SwerveModule backRight = new SwerveModule(20, 18, 19,0.128418, true, "back right", 0.015, 2.46);
  private final SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(dimensionTwo,dimension), new Translation2d(dimensionTwo,-dimension), new Translation2d(-dimensionTwo,dimension), new Translation2d(-dimensionTwo,-dimension));
   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  /* private final MutVoltage m_appliedVoltage = Volts.mutable(0);        
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final LinearVelocity  m_velocity = MetersPerSecond.of(0);
  private int limeLightPipeLine = 0; */


 /*  boolean keepTurning = false;
  double robotOffset = 0;
 // PIDController pidController = new PIDController(0.007, 0, 0);
  PIDController pidController = new PIDController(0.007,0,0);
  SwerveDrivePoseEstimator odometry;

  PIDController yPidController = new PIDController(4/12*4.86, 0,0);
  PIDController xPidController = new PIDController(4/12*4.86, 0,0);
  PIDController turningPidController = new PIDController(0.007, 0,0); */

  
  
  //private Field2d field = new Field2d();
  
  public DriveSub() {
    /* positionPidController.setTolerance(0.1);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(4);
    turningPidController.setTolerance(0.1);
    turningPidController.enableContinuousInput(-180, 180);
    turningPidController.setTolerance(4); */
   

    /* try{
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetOdometry, 
        this::getSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0)
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses)); */

    


  }

  

  
}
