package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Shooter_Subsystem extends SubsystemBase {
    
 TalonFX m_LeftShooter = new TalonFX(RobotConstants.ShooterLcan);
 TalonFX m_RightShooter = new TalonFX(RobotConstants.ShooterRcan);
   Follower  right_Follower;
     TalonFXConfiguration config;

    public Shooter_Subsystem(){

     //m_RightShooter.setControl(new Follower(m_LeftShooter.getDeviceID(), null));
    //  right_Follower = new Follower(RobotConstants.ShooterLcan, null);
    //     m_RightShooter.setControl(right_Follower);

         


       config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        // "Nobody uses I" apparently, so dont set it.
        config.Slot0.kD = 0.0087045;
        config.Slot0.kS = 0.034662;
        config.Slot0.kV = 2.00919;
        config.Slot0.kA = 0.0015569;

        // Zero means no limit to cruise velocity
        config.MotionMagic.MotionMagicCruiseVelocity = 1000; // changed from 250 (12 inches / second) * (1 sprocket
                                                             // rotation / 1.8 inches
                                                             // ) * (90 motor rotations / sprocket rotation) = ~191
                                                             // motor
                                                             // roations / second.8 inches /
        config.MotionMagic.MotionMagicAcceleration = config.MotionMagic.MotionMagicCruiseVelocity * 2 * 100; //
        // .5
        // seconds
        // to
        // reach
        // full
        // speed
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10 * 50; // spread jerk over
                                                                                                   // .1
                                                                                           

      m_LeftShooter.getConfigurator().apply(config);

    }

    //  public void setshooter(AngularVelocity speed) {
    //     final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle0;
    //     m_LeftShooter.setControlPrivate(m_request.); }

   
    public final void  setspeed(double velocityVoltage){
         MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(velocityVoltage);
        m_LeftShooter.setControl(m_request);
        m_RightShooter.setControl(m_request);
        
    }
        private int shswitch = 1; 
 public Command shooter_run(double speed){
        m_LeftShooter.set(-speed *shswitch);
        m_RightShooter.set(speed * shswitch);
     return null;}
  // public Command shturn(){
  //   if(shswitch == 1){
  //     shswitch =0;
  //   }
  //   else{
  //     shswitch =1;
  //   }
  //   return null;
  // }

     //DEL
    // // public Command stopShooter(){
    // //     m_LeftShooter.set(0);
    // //     m_RightShooter.set(0);
    // //     return null;
    // }
}
