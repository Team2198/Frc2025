// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase {


  private final SparkMax motorRight    = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax motorLeft    = new SparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder encoderRight  = motorRight.getEncoder();
  private final RelativeEncoder encoderLeft  = motorLeft.getEncoder();
  
   public final Trigger atMin = new Trigger(()->getLinearPosition().isNear(Inches.of(5), Inches.of(1)));
  public final Trigger atMax = new Trigger(() -> getLinearPosition().isNear(Inches.of(40),
                                                                            Inches.of(1)));
  SparkMaxConfig configRight = new SparkMaxConfig();
  SparkMaxConfig configLeft= new SparkMaxConfig();
  /** Creates a new Elevator. */
   // SysId Routine and seutp
   double totalVoltage = 0;
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance       m_distance       = Meters.mutable(0);
  
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);

  PIDController elevatorController = new PIDController(0.4,0,0);
  
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,

          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.kElevatorKp,
                                                                              Constants.kElevatorKi,
                                                                               Constants.kElevatorKd,
                                                                               new Constraints(Constants.ElevatorkMaxVelocity,
                                                                                               m_feedforward.maxAchievableAcceleration(12, Constants.ElevatorkMaxVelocity)));






  // SysID Routine
  private final SysIdRoutine      m_sysIdRoutine   =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.per(Second).of(1),
                                  Volts.of(7),
                                  Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              voltage -> {
                motorRight.setVoltage(voltage);
                motorLeft.setVoltage(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("elevator right")
                   .voltage(
                       m_appliedVoltage.mut_replace(
                           motorRight.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(getHeightMeters(),
                                                          Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                   .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
                                                          MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
                
                log.motor("elevator left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            motorLeft.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getHeightMeters(),
                                                           Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                    .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
                                                           MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
                  
              
              
              
              
              
              
              
              },
              this));

  
  public Elevator() {
    configRight
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    configRight.encoder
    //1:20 ratio
    
    .positionConversionFactor(0.05*Units.inchesToMeters(1.757)*Math.PI*2)
    .velocityConversionFactor(0.05*Units.inchesToMeters(1.757)*Math.PI*2);
    configRight.signals.primaryEncoderPositionPeriodMs(20);
    configRight.signals.primaryEncoderVelocityPeriodMs(20);
    configRight.smartCurrentLimit(40);
    
    motorRight.configure(configRight, null, null);
    configLeft
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    configLeft.encoder
    //1:20 ratio
    .positionConversionFactor(0.05*Units.inchesToMeters(1.757)*Math.PI*2)
    .velocityConversionFactor(0.05*Units.inchesToMeters(1.757)*Math.PI*2);
    configLeft.signals.primaryEncoderPositionPeriodMs(20);
    configLeft.signals.primaryEncoderVelocityPeriodMs(20);
    configLeft.smartCurrentLimit(40);
    motorLeft.configure(configLeft, null, null);
    elevatorController.setTolerance(1);

    encoderRight.setPosition(0);
    encoderLeft.setPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("velocity elevator", getVelocityMetersPerSecond());
    //SmartDashboard.putNumber("Elevator Height", getHeightMeters());
    getHeightMeters();
    
  }


  double getHeightMeters(){
    double position = (encoderRight.getPosition()+encoderLeft.getPosition())/2;
    SmartDashboard.putNumber("raw elevator height", position);
    SmartDashboard.putNumber("raw elevator height", position);
    SmartDashboard.putNumber("left elevator height", encoderLeft.getPosition());
    SmartDashboard.putNumber("right elevator height", encoderRight.getPosition());
    SmartDashboard.putNumber("raw elevator height inches", Units.metersToInches(position));
    SmartDashboard.putNumber("raw elevator height inches", Units.metersToInches(position));
    SmartDashboard.putNumber("left elevator height inches", Units.metersToInches(encoderLeft.getPosition()));
    SmartDashboard.putNumber("right elevator height inches", Units.metersToInches(encoderRight.getPosition()));
    return position;



  }

  double getVelocityMetersPerSecond(){
    return (encoderRight.getVelocity()+encoderLeft.getVelocity())/2;
  }

   public Command runSysIdRoutine(){
    return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE"));
  }


  public Distance   getLinearPosition()
  {
    return Meters.of(getHeightMeters());
  }

  public void setVoltage(double voltage){
    motorRight.setVoltage(voltage);
    motorLeft.setVoltage(voltage);
    //SmartDashboard.putNumber("ele voltage", voltage);
    
  }

  public void setRight(double voltage){
    motorRight.setVoltage(voltage);
    
    

  }

  public void setLeft(double voltage){
    motorLeft.setVoltage(voltage);
    
    
    
  }

  public void setHeight(double height){
    //double voltage = m_controller.calculate(getHeightMeters(), height);
    //SmartDashboard.putNumber("pid voltage", voltage);
    //voltage+= m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),m_controller.getSetpoint().velocity);
    //SmartDashboard.putNumber("feedforward", m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),m_controller.getSetpoint().velocity));
    //SmartDashboard.putNumber("Elevator voltage", voltage);
    double voltage = elevatorController.calculate(Units.metersToInches(getHeightMeters()),height)+0.3;
    //totalVoltage+=voltage;
    setVoltage(voltage);
  }


  public PIDController getPidController(){
    return elevatorController;
  }
}
