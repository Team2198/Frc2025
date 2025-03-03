// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

//test commit

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
  }

   public static class TeleOp{
    public static double robotRadius = Units.inchesToMeters(27/2);
    public static final double maxSpeed = 4.86;
    public static final double maxTurningRad= maxSpeed/(Math.sqrt(2*robotRadius*robotRadius));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 1000);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

public static final double ElevatorkMaxVelocity = 0.1;



public static double kElevatorKp;
public static double kElevatorKi;
public static double kElevatorKd;

public static double kElevatorkS;

public static double kElevatorkG;

public static double kElevatorkV;

public static double kElevatorkA;


}
