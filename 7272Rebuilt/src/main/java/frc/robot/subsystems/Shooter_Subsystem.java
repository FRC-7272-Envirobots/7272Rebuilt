package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Shooter_Subsystem extends SubsystemBase {
    
 TalonFX m_LeftShooter = new TalonFX(RobotConstants.ShooterLcan);
 TalonFX m_RightShooter = new TalonFX(RobotConstants.ShooterRcan);
   Follower  right_Follower;
     TalonFXConfiguration config;

    public Shooter_Subsystem(){

      //  right_Follower = new Follower(m_RightShooter.getDeviceID(), null);`


       //  m_RightShooter.setControl(right_Follower);


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
                                                                                           


    }

    //  public void setshooter(AngularVelocity speed) {
    //     final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle0;
    //     m_LeftShooter.setControlPrivate(m_request.); }

   
    public final void  setspeed(double velocityVoltage){
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(velocityVoltage);
    }
        
 public Command spinup(){
        m_LeftShooter.set(-0.8);
        m_RightShooter.set(0.8);
     return null;}

    public Command stopShooter(){
        m_LeftShooter.set(0);
        m_RightShooter.set(0);
        return null;
    }
}
