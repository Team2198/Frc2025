// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


  private final SparkFlex        m_motor    = new SparkFlex(1, MotorType.kBrushless);
  private final RelativeEncoder m_encoder  = m_motor.getEncoder();

   public final Trigger atMin = new Trigger(()->getLinearPosition().isNear(Inches.of(4), Inches.of(4)));
  public final Trigger atMax = new Trigger(() -> getLinearPosition().isNear(Inches.of(4),
                                                                            Inches.of(3)));
  SparkMaxConfig config = new SparkMaxConfig();
  /** Creates a new Elevator. */
   // SysId Routine and seutp
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance       m_distance       = Meters.mutable(0);
  private final MutAngle          m_rotations      = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);



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
                                                                                               Constants.ElevatorkMaxAcceleration));





  // SysID Routine
  private final SysIdRoutine      m_sysIdRoutine   =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.per(Second).of(1),
                                  Volts.of(7),
                                  Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              m_motor::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("elevator")
                   .voltage(
                       m_appliedVoltage.mut_replace(
                           m_motor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(getHeightMeters(),
                                                          Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                   .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
                                                          MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
              },
              this));

  
  public Elevator() {
    config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Height", getHeightMeters());
    
  }


  double getHeightMeters(){
    
    return m_encoder.getPosition();



  }

  double getVelocityMetersPerSecond(){
    return m_encoder.getVelocity();
  }

   public Command runSysIdRoutine(){
    return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE"));
  }


  public Distance getLinearPosition()
  {
    return Meters.of(getHeightMeters());
  }

  public void setVoltage(double voltage){
    m_motor.setVoltage(voltage);
    Shuffleboard.getTab("Elevator").add("Voltage", voltage);
  }

  public void setHeight(double height){
    double voltage = m_controller.calculate(getHeightMeters(), height);
    voltage+= m_feedforward.calculate(m_controller.getSetpoint().velocity);
    setVoltage(voltage);
  }
}
