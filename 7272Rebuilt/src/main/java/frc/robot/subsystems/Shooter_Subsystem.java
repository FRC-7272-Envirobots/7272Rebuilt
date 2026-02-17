package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Shooter_Subsystem extends SubsystemBase {
    
 TalonFX m_LeftShooter = new TalonFX(RobotConstants.ShooterLcan);
 TalonFX m_RightShooter = new TalonFX(RobotConstants.ShooterRcan);
   // Follower  right_Follower;
     // TalonFXConfiguration config;

    public Shooter_Subsystem(){

       // right_Follower = new Follower(m_RightShooter.getDeviceID(), true);
       // m_RightShooter.setControl(right_Follower);
    }
 public Command spinup(){
        m_LeftShooter.set(-0.4);
        m_RightShooter.set(0.4);
     return null;}

    public Command stopShooter(){
        m_LeftShooter.set(0);
        m_RightShooter.set(0);
        return null;
    }
}
