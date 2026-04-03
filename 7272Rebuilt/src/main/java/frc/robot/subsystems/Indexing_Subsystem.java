package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Indexing_Subsystem extends SubsystemBase{

    SparkMax m_Spindexer = new SparkMax(RobotConstants.spindexerCan, MotorType.kBrushless);
   
    TalonFX m_indexer = new TalonFX(RobotConstants.IndexerCan);

    public int spswitch =1;
    public int idswitch =1;
    public Command spindexer_run(double speed ){
        m_Spindexer.set(-speed * spswitch);
        return null;
    }
    // public Command spturn (){
    //         if (spswitch == 1){
    //             spswitch = 0;
    //         }
    //         else{
    //             spswitch =1;
    //         }
    //     return null;
    // }
    

    public Command indexer_run(double speed){
        m_indexer.set(speed *idswitch);
        return null;
    }
      public Command feed(double spispeed,double indspeed){
        m_Spindexer.set(spispeed);
        m_indexer.set(indspeed);
        return null;
    }
    public Command feedw(double spispeed,double indspeed){
        Commands.waitSeconds(0.5)
       .andThen(feed(spispeed, indspeed));
      
        return null;
    }
    // public Command idturn(){
    //     if (idswitch == 1){
    //         idswitch =0;
    //     }
    //     else{
    //         idswitch =1;
    //     }
    //     return null;
    // }
  
    
    
}
