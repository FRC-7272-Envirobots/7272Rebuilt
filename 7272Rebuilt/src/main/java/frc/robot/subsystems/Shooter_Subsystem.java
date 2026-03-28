package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Shooter_Subsystem extends SubsystemBase {
    
 TalonFX m_LeftShooter = new TalonFX(frc.robot.Constants.RobotConstants.ShooterLcan);
 TalonFX m_RightShooter = new TalonFX(frc.robot.Constants.RobotConstants.ShooterRcan);
 Follower  right_Follower;
 TalonFXConfiguration config;
 public double shootervel =0;

    public Shooter_Subsystem(){

     //m_RightShooter.setControl(new Follower(m_LeftShooter.getDeviceID(), null));
    //  right_Follower = new Follower(RobotConstants.ShooterLcan, null);
    //     m_RightShooter.setControl(right_Follower);

         


       config = new TalonFXConfiguration();
        config.Slot0.kP = 0.3;
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
         MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(velocityVoltage*-1);
        m_LeftShooter.setControl(m_request);
        m_RightShooter.setControl(m_request);
        
    }
     public final void  distspeed(double shootervel){
         MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(shootervel-1);
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
   

     
  
   public double getshooterspeed(double distToRobot){
     // v\left(x\right)=\sqrt{\frac{x^{2}\cdot9.806}{x\sin\left(2\cdot a_{1}\right)-2\cdot h_{1}\cos\left(a_{1}\right)^{2}}}
      shootervel = Math.sqrt( 
        (Math.pow(distToRobot,2)*LauncherConstants.gravity)/
        (distToRobot*Math.sin(2*LauncherConstants.luchagl)-(2*LauncherConstants.shooterht*(Math.pow(Math.cos(distToRobot),2))))
         
         );
      

      return shootervel;
   }
}
