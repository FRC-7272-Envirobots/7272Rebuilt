package frc.robot.subsystems;

import java.sql.Time;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;



public class Intake_Subsystem extends SubsystemBase {

    public boolean down=false;
   // SparkMax m_intakeArm = new SparkMax(RobotConstants.ArmCan, MotorType.kBrushless);
   // SparkMax m_intakespin = new SparkMax(RobotConstants.IntakeCan,MotorType.kBrushless);
    TalonFX m_intakeArm = new TalonFX(RobotConstants.ArmCan);
    TalonFX m_intakespin = new TalonFX(RobotConstants.IntakeCan);
    public Intake_Subsystem(){

    }

    public Command intake_down(){
        if (down == false){
            m_intakeArm.set(0.2);
           m_intakeArm.set(0);
        }
        else{
            m_intakeArm.set(0);
        }
        return null;
    }
    public Command wiggle(){
        if (down == true){
            m_intakeArm.set(-0.2);
            m_intakeArm.set(0);
        }
        return null;
    }
    public Command intake_run(){
        m_intakespin.set(0.6);
        return null;
    }
    public Command intake_stop(){
        m_intakespin.set(0);
        return null;
    }
    
}
