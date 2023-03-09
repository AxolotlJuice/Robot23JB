package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static Team4450.Robot23.Constants.*;

public class Arm extends SubsystemBase{

    private double                  radius, radians, targetExtend, targetRotate, currentSpeed, elapsedTime;

    private SynchronousPID          pid;

    //channel tbd
    private CANSparkMax             armMotor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);

    private DigitalInput    limitSwitch = new DigitalInput(ARM_SWITCH);

    private Command			        command = null;
    private SequentialCommandGroup  commands = null; 

    public Arm(){
        Util.consoleLog("Arm created!");
    }

    public void initialize(){
        
        armMotor.getEncoder().getPosition();

        Util.consoleLog();
    }

    public void periodic(){
        
    }

    
    public void setArmCounts(int counts, double startSpeed){
        
        pid.setSetpoint(counts);

        while(armMotor.getEncoder().getVelocity() != 0.0){
        
            elapsedTime = Util.getElaspedTime();

            currentSpeed = pid.calculate(armMotor.getEncoder().getPosition(), elapsedTime);

            armMotor.set(currentSpeed);
        }
    }
    
    public void setPower(double power){
        armMotor.set(power);
    }

    public CANSparkMax getMotor(){
        return armMotor;
    }

    public void setToPreset(){
        
    }

    /* 
    public void setArmPose(double power, Pose2d targetPose){
        
        //Uses math to determine the robot's radius and radians 
        radius = targetPose.getX()/Math.acos(targetPose.getX());
        radians = Math.asin(targetPose.getY()/radius);

        //set position coversion factor by measure the distance extended after one revolution
        targetExtend = armMotor1.getEncoder().getPositionConversionFactor() * radius;

        targetRotate = armMotor2.getEncoder().getCountsPerRevolution() * (radians/(2 * Math.PI));

        commands = new SequentialCommandGroup();

        command = new SimultaneousArmPID(armMotor1, armMotor2, targetExtend, targetRotate);
        commands.addCommands(command);

        commands.schedule();
    }

    public void setArmAction(double ticksRotate, double distExtension){
        
        targetExtend = distExtension;

        targetRotate = ticksRotate;
        
        commands = new SequentialCommandGroup();

        command = new SimultaneousArmPID(armMotor1, armMotor2, targetExtend, targetRotate);
        commands.addCommands(command);

        commands.schedule();
    }
    */

}
