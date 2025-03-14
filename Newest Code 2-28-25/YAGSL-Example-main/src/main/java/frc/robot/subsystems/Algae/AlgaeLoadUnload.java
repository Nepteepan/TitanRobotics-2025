package frc.robot.subsystems.Algae;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.LifterConstants;

public class AlgaeLoadUnload extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  public final SparkMax algaeIntake;
  private double goal = 0;

  private final RelativeEncoder m_encoder;

  public AlgaeLoadUnload() {
    algaeIntake = new SparkMax(AlgaeConstants.ALGAE_INTAKE_PORT, MotorType.kBrushless);
    m_encoder  = algaeIntake.getEncoder();
    SparkMaxConfig AlgaeIntakeConfig = new SparkMaxConfig();
    AlgaeIntakeConfig
        .smartCurrentLimit(LifterConstants.kLifterCurrentLimit)
        .closedLoopRampRate(LifterConstants.kLifterRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-.5, .5);
    AlgaeIntakeConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    AlgaeIntakeConfig.inverted(false);
    algaeIntake.configure(AlgaeIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    seedAlgaeMotorPosition();
  }

  /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void seedAlgaeMotorPosition()
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
    m_encoder.setPosition(goal);
    algaeIntake.setVoltage(AlgaeConstants.ALGAE_INTAKE_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeAlgae(){
    algaeIntake.set(AlgaeConstants.ALGAE_INTAKE_SPEED);
    //check for current spike(when algae )
  }
  public void placeAlgae(){
    algaeIntake.set(AlgaeConstants.ALGAE_PLACE_SPEED);
  }
  public void holdAlgae(){
    algaeIntake.set(AlgaeConstants.ALGAE_HOLD_SPEED);
  }

public Command intakeAlgae(double goal)
  {
    //return new MoveElevator(this, goal);
    return run(() -> intakeAlgae());
  }

  public Command relaseAlgae(double goal)
  {
    //return new MoveElevator(this, goal);
    return run(() -> placeAlgae());
  }

  public Command holdAlgae(double goal)
  {
    //return new MoveElevator(this, goal);
    return run(() -> holdAlgae());
  }
}

