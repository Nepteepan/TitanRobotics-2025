package frc.robot.subsystems;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotMath.Arm;
import frc.robot.RobotMath.Elevator;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class scoringRoutines {
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private SwerveSubsystem swervesubsystem;
    private double Position1elevator = 0;
    private double Position2elevator = .75;
    private double Position3elevator = 2.2;
    private double elevatorloadposition = .35;
    private double elevatorhome = 0;
    private double Position1arm = -28;
    private double Position2arm = -28;
    private double Position3arm = -28;
    private double armhome = 0;



    public scoringRoutines(ElevatorSubsystem elevator, ArmSubsystem arm, SwerveSubsystem swervesubsystem){
        this.elevator = elevator;
        this.arm = arm;
        this.swervesubsystem = swervesubsystem;
    }

    // Sequence 1 Start
    public Command movelevel2() {
        return Commands.sequence(elevator.setGoal(elevatorloadposition).withTimeout(1),
        arm.setGoal(Position1arm).withTimeout(1),
        elevator.setGoal(Position1elevator));
        
    }
    //Sequence 1 End
    //Sequence Home Start
    public Command movehome() {
        return Commands.sequence(elevator.setGoal(elevatorloadposition).withTimeout(1),
        arm.setGoal(armhome).withTimeout(1),
        elevator.setGoal(elevatorhome));
        
    }
    //Sequence Home End
    //Sequence 2 Start
    public Command movelevel3() {
        return Commands.sequence(elevator.setGoal(Position2elevator).withTimeout(1),
        arm.setGoal(Position2arm));
        
        
    }
    //Sequence 3 Start
    public Command movelevel4() {
        return Commands.sequence(elevator.setGoal(Position3elevator).withTimeout(1),
        arm.setGoal(Position3arm));
        
        
    }
    //Sequence 3 End
    //Sequence Load Start
    public Command moveloadposition() {
        return Commands.sequence(elevator.setGoal(elevatorloadposition).withTimeout(1),
        arm.setGoal(armhome));
    }
     //Sequence Load End
     public Command scorelevel4() {
        return Commands.sequence(arm.setGoal(armhome).withTimeout(1),
        swervesubsystem.driveToDistanceCommand(-.25, .25),
        elevator.setGoal(elevatorloadposition));
    }
}
