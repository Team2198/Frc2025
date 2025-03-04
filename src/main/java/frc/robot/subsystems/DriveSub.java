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
  private final SwerveModule frontLeft = new SwerveModule(18, 19, 25,0.376709, true,"front left", 0.015, 2.4585);
  private final SwerveModule frontRight = new SwerveModule(16, 17, 23,0.323242, true,"front right",0.015, 2.4691);
  AHRS gyro = new AHRS(null);
  PIDController positionPidController = new PIDController(4,0,0);
  int counter=0;
  double dimension = Units.inchesToMeters(27/2);
  double dimensionTwo = Units.inchesToMeters(32/2);
  //private final SwerveModule backLeft = new SwerveModule(20, 21, 26, 0.268311, true, "back left", 0.015, 2.4978);
  private final SwerveModule backLeft = new SwerveModule(20, 21, 26, 0.261963, true, "back left", 0.015, 2.4978);
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(dimensionTwo,dimension), new Translation2d(dimensionTwo,-dimension), new Translation2d(-dimensionTwo,dimension), new Translation2d(-dimensionTwo,-dimension));
  //private final SwerveModule backRight = new SwerveModule(14, 15, 22,0.133301, true, "back right", 0.015, 2.46);
  private final SwerveModule backRight = new SwerveModule(14, 15, 22,0.128418, true, "back right", 0.015, 2.46);
  private final SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
  
   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);        
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final LinearVelocity  m_velocity = MetersPerSecond.of(0);
  
  boolean keepTurning = false;
  double robotOffset = 0;
 // PIDController pidController = new PIDController(0.007, 0, 0);
  PIDController pidController = new PIDController(0.007,0,0);
  SwerveDrivePoseEstimator odometry;

  PIDController yPidController = new PIDController(4/12*4.86, 0,0);
  PIDController xPidController = new PIDController(4/12*4.86, 0,0);
  PIDController turningPidController = new PIDController(0.007, 0,0);

  
  
  private Field2d field = new Field2d();
  
  public DriveSub() {
    positionPidController.setTolerance(0.1);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(4);
    turningPidController.setTolerance(0.1);
    turningPidController.enableContinuousInput(-180, 180);
    turningPidController.setTolerance(4);
    robotRelative(0, 0, 0);

    try{
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
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("yaw",getHeading());
    SmartDashboard.putNumber("robot offset", robotOffset);
    //setAngle(90);

    odometry.update(getRotation2d(), getModulePositions());
  }


  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Rotation2d getRawRotation2d(){
    return Rotation2d.fromDegrees(getRawHeading());
  }



  public void setSpeed(double speed, double turningSpeed){
    for (int i=0;i<modules.length;i++)
    modules[i].setSpeed(speed, turningSpeed);
    //frontRight.setSpeed(speed, turningSpeed);
    //backRight.setSpeed(speed, turningSpeed);
    //backLeft.setSpeed(speed, turningSpeed);
  }

  public double getHeading(){
    //invert gyro yaw reading
    double angle = (gyro.getYaw()*-1);
    angle = angle+robotOffset;
    

    if (angle>180){
      angle = angle-360;
    }

    


   

    return angle;

  }

  public double getRawHeading(){
    double angle = (gyro.getYaw()*-1);
    SmartDashboard.putNumber("raw heading", angle);
    return angle;
  }




 



  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    return modulePositions;
  }

  public double getDrivePosition(){
    double averageDrive = 0;
     for (int i=0;i<modules.length;i++){
      averageDrive+=modules[i].getDrivePosition()/4;
    }
    return averageDrive;
  }

  

  public void zeroYaw(){
    gyro.zeroYaw();
  }


  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    
    setModuleStates(targetSpeeds);
  }

  


  public Pose2d getPose(){
     
    return odometry.getEstimatedPosition();
  }

  public ChassisSpeeds getSpeeds(){
    return kinematics.toChassisSpeeds(getModuleStates());

  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] moduleStates = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    return moduleStates;
  }

  public void resetOdometry(Pose2d pose){
    SmartDashboard.putString("inside odomtry", "yes");
    SwerveModulePosition[] modulePositions = getModulePositions();
    odometry.resetPosition(getRotation2d(), modulePositions, pose);
  }

  public void resetOdometryVision(){
    SwerveModulePosition[] modulePositions = getModulePositions();
    odometry.resetPosition(getRotation2d(), modulePositions, getPose());
  }



   

  


  public double getRotHeading(){
    double angle = (gyro.getYaw()*-1)+robotOffset;
    if (angle>180){
      angle = angle-360;
    }

    else if (angle<180){
      angle = 360+angle;
    }

    return angle;

  }

  public void setModuleStates(ChassisSpeeds speed){
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speed);
    //desaturate wheelspeeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.TeleOp.maxSpeed);
    SmartDashboard.putNumber("module angles2", Math.toDegrees(moduleStates[0].angle.getRadians()));
    SmartDashboard.putNumber("module velocity", (moduleStates[0].speedMetersPerSecond));
   
    for (int i=0; i<modules.length;i++){
      modules[i].setState(moduleStates[i]);
    }

     //backRight.setState(moduleStates[3]);
  }

  public void robotRelative(double xSpeed, double ySpeed, double turningSpeed){
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    SmartDashboard.putNumber("xSpeed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ySpeed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("turning rad", speeds.omegaRadiansPerSecond);
    setModuleStates(speeds);
    
  }

  public void fieldRelative(double xSpeed, double ySpeed, double turningSpeed){
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, new Rotation2d(getHeading()*Math.PI/180));
    SmartDashboard.putNumber("xSpeed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ySpeed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("turning rad", speeds.omegaRadiansPerSecond);
    setModuleStates(speeds);
  }

  public void resetPidController(){
    xPidController.reset();
    yPidController.reset();
    turningPidController.reset();
  }

  

  public void setAngle(double angle){
    for (int i=0;i<modules.length;i++){
      modules[i].setAngle(angle);
      SmartDashboard.putNumber("number", counter);
    }
  } 

  public void stopMotors(){
    setSpeed(0, 0);
  }

  public boolean atSetpoint(){
   return pidController.atSetpoint();
   //return false;
  } 

  //pass the angle that you want the robot to turn to
  public double turnToAngle(double goal){
    
    if (goal>180){
      goal = goal-360;
    }

    double turnSpeed = pidController.calculate(getHeading(), goal);
    turnSpeed*=Constants.TeleOp.maxTurningRad*0.8;

    
    
    //returns true if auto aligned
    return turnSpeed;
    //return 0;
    
  }

  

  public void setRobotRelative(ChassisSpeeds speeds){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    
  }


  public boolean followPath(double target, double angle){
    //setAngle(angle);
    for (int i=0;i<modules.length;i++){
      modules[i].setAngle(angle);
      modules[i].followPath(positionPidController.calculate(getDrivePosition(), target));

    } 
    return positionPidController.atSetpoint();

  }

  public void zeroEncoders(){
    for (int i=0;i<modules.length;i++){
      modules[i].zeroDrive(); 
    } 
  }

  public boolean getLimelightTV(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0) == 1; 
  }

  public double[] getLimelightBotpose(){

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry botPosition = table.getEntry("botpose_targetspace");
    double[] botPose = botPosition.getDoubleArray(new double[6]);
    return botPose;
  }

  public void setOffset(double angle){
    robotOffset = angle;
  }

  public Command setRobotOffset(double angle){
    
    return this.runOnce(()->this.setOffset(angle));
  }

  public Command zeroEncodersCommand(){
    return this.runOnce(()->this.zeroEncoders());
  }

  public void setwheelAngles(double angle){
    for (int i=0;i<modules.length;i++){
      modules[i].setAngle(angle); 
    } 
  }

  public Command setWheelAngleCommand(double angle){
    return this.run(()->this.setwheelAngles(angle));
  }

  public Command visionReset(){
    return this.runOnce(()->this.resetOdometryVision());

  }

  public double findLinearDist(Pose2d target){
    
    double dist = target.minus(getPose()).getTranslation().getDistance(new Translation2d());
    return dist;
    
  }

  public Pose2d reefEndPose(){
    //this transform is the limelight transform
    Transform2d transform = new Transform2d(
    new Translation2d(1.0, 0.5), // Translation (x, y)
    new Rotation2d(Math.toRadians(30)) // Rotation in radians
    );

    // New pose relative to the current pose
    Pose2d newPose = getPose().transformBy(transform);
    return newPose;    
    //use this pose to find distance remaining to reef
  }

  public PathPlannerPath generatePathToReef(){
    //vision logic here
    //important thread: https://www.chiefdelphi.com/t/pathplanners-pathfinding-not-working-for-close-pose-estimation/484786/2
    //https://pathplanner.dev/pplib-create-a-path-on-the-fly.html
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    //The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    //the rotation component should be calculated from x and y components
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
          new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
          new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
    );

    PathConstraints constraints = new PathConstraints(3.0, 10/3, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct

    //alliance logic
    path.preventFlipping = true;

    return path;
  }
  //this method needs the reef pose relative to the current robot pose
  //use limelight data to generate a target pose relative to current robot pose
  //dont forget to take into account that limelight will return camera to target pose
  //we need robot to target pose
  public boolean followPathNew(Pose2d target){

    
    double xSpeed = xPidController.calculate(getPose().getX(), target.getX());
    double ySpeed = yPidController.calculate(getPose().getY(), target.getY());
    double turningSpeed = turningPidController.calculate(getPose().getRotation().getDegrees(), target.getRotation().getDegrees());

    robotRelative(xSpeed,ySpeed,turningSpeed);

    return xPidController.atSetpoint() && yPidController.atSetpoint()&&turningPidController.atSetpoint();

  }

  
}
