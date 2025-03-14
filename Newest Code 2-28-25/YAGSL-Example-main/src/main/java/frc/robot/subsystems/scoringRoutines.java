package frc.robot.subsystems;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMath.Arm;
import frc.robot.RobotMath.Elevator;
import frc.robot.commands.MoveElevator;
import frc.robot.Arm.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.Algae.AlgaeLoadUnload;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class scoringRoutines {
    private AlgaeSubsystem lifter;
    private NeoPositionalPid arm;
    private ElevatorSubsystem elevator;
    private SwerveSubsystem swervesubsystem;
    private AlgaeLoadUnload algaeLoadUnload;
    private double Position2elevator = 0;
    private double Position3elevator = .75;
    private double Position4elevator = 2.4;//was 2.3
    private double elevatorloadposition = .35;
    private double elevatorhome = 0;
    private double Position1arm = 19;
    private double Position2arm = 15;
    private double Position3arm = 15;
    private double armhome = 0;
    private double lifterhome = 0;
    private double lifterhalfway = 1.3;
    private double lifterfull = 4;



    public scoringRoutines(ElevatorSubsystem elevator, NeoPositionalPid arm, SwerveSubsystem swervesubsystem, AlgaeSubsystem lifter, AlgaeLoadUnload algaeLoadUnload){
        this.elevator = elevator;
        this.arm = arm;
        this.swervesubsystem = swervesubsystem;
        this.lifter = lifter;
        this.algaeLoadUnload = algaeLoadUnload;
    }

    public Command RaiseAlgaeArm() {
        SmartDashboard.putNumber("Algae Goal", 0);
        return Commands.sequence(lifter.setGoal(0));
    }

    public Command LowerAlgaeArm() {
        SmartDashboard.putNumber("Algae Goal", 1);
        return Commands.sequence(lifter.setGoal(.85));
    }

    public Command intakeAlgae() {
        SmartDashboard.putNumber("Algae intake", 0);
        return Commands.sequence(algaeLoadUnload.intakeAlgae(0));
    }

    public Command relaseAlgae() {
        SmartDashboard.putNumber("Algae intake", 1);
        return Commands.sequence(algaeLoadUnload.relaseAlgae(0));
    }

    public Command holdAlgae() {
        SmartDashboard.putNumber("Algae intake", 1);
        return Commands.sequence(algaeLoadUnload.holdAlgae(0));
    }

    // Sequence 1 Start
    public Command movelevel2() {
        return Commands.sequence(elevator.setGoal(elevatorloadposition).withTimeout(.5),
        arm.setGoal(Position1arm).withTimeout(1.5),
        elevator.setGoal(Position2elevator));
        
    }
    public Command scorelevel2() {
        return new SequentialCommandGroup(
            arm.setGoal(10),
            new WaitCommand(2.0),
            elevator.setGoal(elevatorloadposition),
            new WaitCommand(2.0),
            arm.setGoal(armhome)
        );
    //     return Commands.sequence(arm.setGoal(-14),
    //     new WaitCommand(0.5)//.withTimeout(.5),
    //     ,arm.setGoal(armhome).//withTimeout(.25),
    //    ,elevator.setGoal(elevatorloadposition));
    }
    //Sequence 1 End
    //Sequence Home Start
    public Command movehome() {
        return Commands.sequence(elevator.setGoal(elevatorloadposition).withTimeout(.25),
        arm.setGoal(armhome).withTimeout(.50),
        elevator.setGoal(elevatorhome));
        
    }

    //Sequence Home End
    //Sequence 2 Start
    public Command movelevel3() {
        return Commands.sequence(elevator.setGoal(Position3elevator).withTimeout(.5),
        arm.setGoal(Position2arm));
        
        
    }
    //Sequence 3 Start
    public Command movelevel4() {
        SmartDashboard.putNumber("Elevator Goal", Position4elevator);
        return Commands.sequence(elevator.setGoal(Position3elevator),
        arm.setGoal(Position3arm), elevator.setGoal(Position4elevator));
        
        
    }
    //Sequence 3 End
    //Sequence Load Start
    public Command moveloadposition() {
        return Commands.sequence(
            new MoveElevator(elevator, elevatorloadposition),
            arm.setGoal(armhome+2)//,new WaitCommand(.5),
        //arm.setGoal(armhome)
        );
    }
     //Sequence Load End
     public Command scorelevel4() {
        return Commands.sequence(arm.setGoal(Position3arm-5).withTimeout(.5),
        //swervesubsystem.driveToDistanceCommand(.25, -.15),
        arm.setGoal(armhome),
        elevator.setGoal(elevatorloadposition));
    }


    public Command hangprep(){
        return Commands.sequence(lifter.setGoal(lifterhalfway));

    }

    public Command hang(){
        return Commands.sequence(lifter.setGoal(lifterfull));

    }

    public Command AutoDrive() {
        return Commands.sequence(swervesubsystem.driveToDistanceCommand(1, 1));
    }
}
