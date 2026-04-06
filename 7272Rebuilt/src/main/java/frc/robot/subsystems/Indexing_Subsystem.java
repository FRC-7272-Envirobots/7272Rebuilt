package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Indexing_Subsystem extends SubsystemBase {

  SparkMax m_Spindexer = new SparkMax(RobotConstants.spindexerCan, MotorType.kBrushless);

  TalonFX m_indexer = new TalonFX(RobotConstants.IndexerCan);

  private Timer timer;

  public Indexing_Subsystem() {
    this.timer = new Timer();

  }

  public Commands feed(double spispeed, double indspeed) {
    m_indexer.set(indspeed);
    m_Spindexer.set(spispeed);
    System.out.println(indspeed);
    return null;
  }

  // public void feed(double spispeed, double indspeed, boolean jitter) {

  // timer.reset();
  // timer.start();
  // System.out.println(timer.get());
  // // handle spindexer
  // m_Spindexer.set(spispeed);
  // int time;
  // // handle indexer
  // if (jitter == true) {

  // System.out.println(Math.round(timer.get()) % 4);
  // if ((Math.round(timer.get()) % 2) == 0) {

  // m_indexer.set(indspeed);
  // System.out.println(".8");

  // } else {

  // m_indexer.set(0);
  // System.out.println("0");

  // }

  // } else {
  // m_indexer.set(indspeed);
  // }

  // }

  public void periodic() {
    // System.out.println(timer.get());
  }

  public Command feedw(double spispeed, double indspeed) {
    // Commands.waitSeconds(0.5)
    // .andThen(feed(spispeed, indspeed));

    return null;
  }

}
