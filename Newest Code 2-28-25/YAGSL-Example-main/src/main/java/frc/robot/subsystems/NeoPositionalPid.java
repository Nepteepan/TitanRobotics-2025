package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoPositionalPid extends SubsystemBase{
    String className = this.getClass().getSimpleName();
    public double CurrentRotationPosition;
    public double setpointGoal = 0;
    public int MotorCanId = 52;

    private SparkMax m_rotate1650 = new SparkMax(MotorCanId, MotorType.kBrushless);

    private final RelativeEncoder m_Rotate1650Encoder = m_rotate1650.getEncoder();
    private final SparkClosedLoopController  m_rotate1650PID = m_rotate1650.getClosedLoopController();

    public double Proportional = 0.09;
    public double Integral = 0;
    public double Derivative = 0.001;

    public double minposition = 0;
    public double maxPosition = 22;
    
    private SparkMaxConfig motorConfig = new SparkMaxConfig();

    public NeoPositionalPid(){
        SetPidToConfig();
        m_Rotate1650Encoder.setPosition(minposition);

        
        //not breaking the robot with more numbers!
        motorConfig.inverted(false);
        motorConfig.softLimit.forwardSoftLimitEnabled(true);
        motorConfig.softLimit.forwardSoftLimit(maxPosition);
        motorConfig.softLimit.reverseSoftLimitEnabled(true);
        motorConfig.softLimit.reverseSoftLimit(minposition);


        //Current Limits
        motorConfig.smartCurrentLimit(20);
        m_rotate1650PID.setReference(minposition, ControlType.kPosition);

        PushConfigToController();
    

        //Smart dashboard

        SmartDashboard.putNumber(className +" P Gain", Proportional);
        SmartDashboard.putNumber(className +" I Gain", Integral);
        SmartDashboard.putNumber(className +" D Gain", Derivative);

}
public void PushConfigToController()
{
    m_rotate1650.configure(motorConfig,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
}


    @Override
    public void periodic() {
    EncoderData();
    double Neo1650CurrentDraw = m_rotate1650.getOutputCurrent();

    SmartDashboard.putNumber("setPointGoal", setpointGoal);
    SmartDashboard.putNumber(className +"1650 Amperage", Neo1650CurrentDraw);
    SmartDashboard.putNumber(className +"CurrentPosition", CurrentRotationPosition);

    double p = SmartDashboard.getNumber(className +" P Gain", 0);
    double i = SmartDashboard.getNumber(className +" I Gain", 0);
    double d = SmartDashboard.getNumber(className +" D Gain", 0);
          
    if((p != Proportional)) { Proportional = p; SetPidToConfig(); PushConfigToController();}
    if((i != Integral)) { Integral = i; SetPidToConfig();PushConfigToController(); }
    if((d != Derivative)) { Derivative = d; SetPidToConfig(); PushConfigToController();}

    }

    private void SetPidToConfig() {
        motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(Proportional)
        .i(Integral)
        .d(Derivative)
        .outputRange(-1, 1);
    }

    public void setSetpointToPosition(double ToPosition) {
    setpointGoal = ToPosition;
    m_rotate1650PID.setReference(setpointGoal, ControlType.kPosition);}

    public double firstposition = -10;
    public InstantCommand GotoFirstPosition(){
    return new InstantCommand(()->{
        System.out.println("Going to firstpositon");
        setSetpointToPosition(firstposition);
    });}

    public InstantCommand GotoZero(){
    return new InstantCommand(()->{
        System.out.println("goin to 0");
        setSetpointToPosition(0);
    });
}

    public Command gotoposition(double position)
    {
        return new InstantCommand(()->{
            System.out.println("goin to " + position);
            setSetpointToPosition(position);
        });
    }


    public void EncoderData(){
        CurrentRotationPosition = m_Rotate1650Encoder.getPosition();
    }


}
