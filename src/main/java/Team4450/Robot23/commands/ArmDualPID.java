package Team4450.Robot23.commands;

import com.revrobotics.CANSparkMax;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class ArmDualPID extends CommandBase  {
    
    private SynchronousPID          pid1 = new SynchronousPID(0, 0, 0);
    private SynchronousPID          pid2 = new SynchronousPID(0, 0, 0);

    private CANSparkMax             armMotor1;
    private CANSparkMax             armMotor2;

    private double                  voltsExtend;
    private double                  voltsRotate;
    private double                  startTime;
    private double                  tempTime;
    private double                  elapsedTime;
    private double                  targetVal1;
    private double                  targetVal2;

    public ArmDualPID(CANSparkMax armMotor1, CANSparkMax armMotor2, double targetVal1, double targetVal2){
        this.armMotor1 = armMotor1;
        this.armMotor2 = armMotor2;
        this.targetVal1 = targetVal1;
        this.targetVal2 = targetVal2;
    }

    public void initialize(){
        //below values are temporary
        pid1.setOutputRange(0.0, 0.0);
        pid2.setOutputRange(0.0, 0.0);

        pid1.setSetpoint(targetVal1);
        pid2.setSetpoint(targetVal2);

        startTime = Util.timeStamp();
        tempTime = Util.timeStamp();

    }
    
    public void execute(boolean intrrupted){
        elapsedTime = Util.getElaspedTime(tempTime);

        voltsExtend = 0.5 * pid1.calculate(armMotor1.getEncoder().getPosition(), tempTime);
        voltsRotate = 0.5 * pid2.calculate(armMotor2.getEncoder().getPosition(), tempTime);

        armMotor1.set(voltsExtend);
        armMotor2.set(voltsRotate);
    }

    public boolean isFinished(){
        return (armMotor1.getEncoder().getVelocity() == 0.0) && (armMotor2.getEncoder().getVelocity() == 0.0);
    }

    public void end(){
        
    }
}
