package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Indexing_Subsystem extends SubsystemBase{

    SparkMax m_Spindexer = new SparkMax(RobotConstants.spindexerCan, MotorType.kBrushless);
    
    public Command spindexer_run(){
        m_Spindexer.set(-0.8);
        System.out.println("spining indexer");
        return null;
    }
    public Command spindexer_stop(){
        m_Spindexer.set(0);
        return null;
    }
    
}
