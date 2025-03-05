package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class MoveElevator extends Command {
  // The subsystem the command runs on
  private final ElevatorSubsystem m_ElevatorSubsystem;
    private double wantedposition =0;
  public MoveElevator(ElevatorSubsystem subsystem,double _wantedposition) {
    m_ElevatorSubsystem = subsystem;
    addRequirements(m_ElevatorSubsystem);
    wantedposition =_wantedposition;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_ElevatorSubsystem.reachGoal(wantedposition);
  }

  public double movetolerance = .06;
  @Override
  public boolean isFinished() {
    return  m_ElevatorSubsystem.atHeight(wantedposition, movetolerance).getAsBoolean();
  }
  @Override
  public void end(boolean Interuppted) {
    m_ElevatorSubsystem.HoldPosition();
    return;
  }
}
