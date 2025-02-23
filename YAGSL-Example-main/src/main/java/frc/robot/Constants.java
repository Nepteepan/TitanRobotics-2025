// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotMath.Arm;
import swervelib.math.Matter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ArmConstants
  {

    // The P gain for the PID controller that drives this arm.
    public static final double kArmKp                     = 2.0691;
    public static final double kArmKi                     = 0;
    public static final double kArmKd                     = 0.0;
    public static final Angle  kArmAllowedClosedLoopError = Arm.convertAngleToSensorUnits(Degrees.of(0.01));

    public static final double  kArmReduction                   = 200;
    public static final double  kArmMass                        = 8.0; // Kilograms
    public static final double  kArmLength                      = Inches.of(72).in(Meters);
    public static final Angle   kArmStartingAngle               = Degrees.of(0);
    public static final Angle   kMinAngle                       = Degrees.of(-75);
    public static final Angle   kMaxAngle                       = Degrees.of(255);
    public static final double  kArmRampRate                    = 0.5;
    public static final Angle   kArmOffsetToHorizantalZero      = Rotations.of(0);
    public static final boolean kArmInverted                    = false;
    public static final double  kArmMaxVelocityRPM              = Arm.convertAngleToSensorUnits(Degrees.of(90)).per(
        Second).in(RPM);
    public static final double  kArmMaxAccelerationRPMperSecond = Arm.convertAngleToSensorUnits(Degrees.of(180)).per(
                                                                         Second).per(Second)
                                                                     .in(RPM.per(Second));
    public static final int     kArmStallCurrentLimitAmps       = 40;

    public static final double kArmkS = 0; // volts (V)
    public static final double kArmkG = 0; // volts (V)
    public static final double kArmKv = 0; // volts per velocity (V/RPM)
    public static final double kArmKa = 0; // volts per acceleration (V/(RPM/s))


  }

  public static class ElevatorConstants
  {


    public static final double kElevatorKp = 80;
    public static final double kElevatorKi = 62;
    public static final double kElevatorKd = .3;

    public static final double kElevatorkS = 0.01964; // volts (V)
    public static final double kElevatorkV = 3.894; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.173; // volt per acceleration (V/(m/s²))
    public static final double kElevatorkG = 0.91274; // volts (V)

    public static final double kElevatorGearing    = 5;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1);
    public static final double kCarriageMass       = 4.0; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final Distance kStartingHeightSim = Meters.of(0);
    public static final Distance kMinElevatorHeight = Meters.of(0.0);
    public static final Distance kMaxElevatorHeight = Meters.of(1);


    public static double kElevatorRampRate = 0.6;
    public static int    kElevatorCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(3.5).per(Second).per(Second).in(MetersPerSecondPerSecond);
  }
}
