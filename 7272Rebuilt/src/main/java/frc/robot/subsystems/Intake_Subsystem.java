package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Intake_Subsystem extends SubsystemBase {

  public boolean down = false;
  // SparkMax m_intakeArm = new SparkMax(RobotConstants.ArmCan,
  // MotorType.kBrushless);
  // SparkMax m_intakespin = new
  // SparkMax(RobotConstants.IntakeCan,MotorType.kBrushless);
  TalonFX m_intakeArm = new TalonFX(RobotConstants.ArmCan);
  TalonFX m_intakespin = new TalonFX(RobotConstants.IntakeCan);

  public Intake_Subsystem() {

  }

  public int inswitch = 1;
  // public Command intake_arm(double speed){
  // if (down == false){
  // m_intakeArm.set(speed);
  // down = true;
  // }
  // else{
  // m_intakeArm.set(-speed);
  // }
  // return null;
  // }

  public Command armmove(double speed) {
    m_intakeArm.set(speed);
    return null;
  }

  public Command intake_run(double speed) {
    m_intakespin.set(speed * inswitch);
    return null;
  }

  // public Command inturn(){
  // if (inswitch ==1){
  // inswitch = 0;
  // }
  // else{
  // inswitch = 1;
  // }
  // return null;
  // }

}
