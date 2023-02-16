package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import Team4450.Lib.SynchronousPID;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SimultaneousArmPID;

public class Arm extends SubsystemBase{

    private double                  radius;
    private double                  radians;
    private double                  targetExtend;
    private double                  targetRotate;

    //channel tbd
    private CANSparkMax             armMotor1 = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    //channel tbd
    private CANSparkMax             armMotor2 = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    private Command			        command = null;
    private SequentialCommandGroup  commands = null;

    public Arm(){
        
    }

    public void initialize(){
        armMotor1.getEncoder().getPosition();
    }

    public void periodic(){
        
    }

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

}
