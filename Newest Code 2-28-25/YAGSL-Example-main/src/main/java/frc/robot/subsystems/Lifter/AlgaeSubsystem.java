
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lifter;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.LifterConstants;
import frc.robot.RobotMath.Elevator;

public class AlgaeSubsystem extends SubsystemBase
{

  public final Trigger atMin = new Trigger(() -> getLinearPosition().isNear(LifterConstants.kMinLifterHeight,
                                                                            Inches.of(3)));
  public final Trigger atMax = new Trigger(() -> getLinearPosition().isNear(LifterConstants.kMaxLifterHeight,
                                                                            Inches.of(3)));


  // This gearbox represents a gearbox containing 1 Neo
  //private final DCMotor m_elevatorGearbox = DCMotor.getNEO(52);

  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          LifterConstants.kLifterkS,
          LifterConstants.kLifterkG,
          LifterConstants.kLifterkV,
          LifterConstants.kLifterkA);
  private final SparkMax        m_motor    = new SparkMax(58, MotorType.kBrushless);
  private final RelativeEncoder m_encoder  = m_motor.getEncoder();


  private final ProfiledPIDController m_controller = new ProfiledPIDController(LifterConstants.kLifterKp,
                                                                               LifterConstants.kLifterKi,
                                                                               LifterConstants.kLifterKd,
                                                                               new Constraints(LifterConstants.kMaxVelocity,
                                                                               LifterConstants.kMaxAcceleration));

  

  // SysId Routine and seutp
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance       m_distance       = Meters.mutable(0);
  //private final MutAngle          m_rotations      = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);
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
                log.motor("lifter")
                   .voltage(
                       m_appliedVoltage.mut_replace(
                           m_motor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(getHeightMeters(),
                                                          Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                   .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
                                                          MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
              },
              this));

  /**
   * Subsystem constructor.
   */
  public AlgaeSubsystem()
  {
    SmartDashboard.putNumber(className +" setPointGoal", 0);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(LifterConstants.kLifterCurrentLimit)
        .closedLoopRampRate(LifterConstants.kLifterRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim

    seedElevatorMotorPosition();
  }


  /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void seedElevatorMotorPosition()
  {
    m_encoder.setPosition(0);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    SmartDashboard.putNumber(className +" setPointGoal", goal);
    double voltsOut = MathUtil.clamp(
        m_controller.calculate(getHeightMeters(), goal) +
        m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                              m_controller.getSetpoint().velocity), -7, 7);
    m_motor.setVoltage(voltsOut);
  }
  public double voltstoHoldPosition =.4;
  public void HoldPosition()
  {

    m_motor.setVoltage(voltstoHoldPosition);
  }

  /**
   * Runs the SysId routine to tune the Arm
   *
   * @return SysId Routine command
   */
  public Command runSysIdRoutine()
  {
    return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE"));
  }


  /**
   * Get Elevator Velocity
   *
   * @return Elevator Velocity
   */
  public LinearVelocity getLinearVelocity()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getVelocity())).per(Minute);
  }

  public void RunMotor(double speed) {
    m_motor.set(speed);
  }
  
  /**
   * Get the height of the Elevator
   *
   * @return Height of the elevator
   */
  public Distance getLinearPosition()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getPosition()));
  }

  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeightMeters()
  {
    return (m_encoder.getPosition() / LifterConstants.kLifterGearing) *
           (2 * Math.PI * LifterConstants.kLifterDrumRadius);
  }

  /**
   * The velocity of the elevator in meters per second.
   *
   * @return velocity in meters per second
   */
  public double getVelocityMetersPerSecond()
  {
    return ((m_encoder.getVelocity() / 60)/ LifterConstants.kLifterGearing) *
           (2 * Math.PI * LifterConstants.kLifterDrumRadius);
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeightMeters(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    //return new MoveElevator(this, goal);
    return run(() -> reachGoal(goal));
  }

  public Command increaseGoal(double goal)
  {
    return run(() -> setGoal(getHeightMeters() + goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor.set(0.0);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry()
  {
  }
  String className = this.getClass().getSimpleName();

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber(className +" appliedVoltage", m_motor.getAppliedOutput());
  
    SmartDashboard.putNumber(className +" 1650 Amperage", m_motor.getOutputCurrent());
    SmartDashboard.putNumber(className +" HeightMeters", getHeightMeters());
  }
}
