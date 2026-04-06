package frc.robot.subsystems;

import java.util.Set;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Indexing_Subsystem extends SubsystemBase {

  private static final String PREFERENCE_JITTER_ON_SECONDS_KEY = "Indexer/Jitter On Duration (seconds)";
  private static final double PREFERENCE_JITTER_ON_SECONDS_DEFAULT = 1.0;

  private static final String PREFERENCE_JITTER_OFF_SECONDS_KEY = "Indexer/Jitter Off Duration (seconds)";
  private static final double PREFERENCE_JITTER_OFF_SECONDS_DEFAULT = 0.2;

  SparkMax m_Spindexer = new SparkMax(RobotConstants.spindexerCan, MotorType.kBrushless);

  TalonFX m_indexer = new TalonFX(RobotConstants.IndexerCan);

  public Indexing_Subsystem() {
    Preferences.initDouble(PREFERENCE_JITTER_ON_SECONDS_KEY, PREFERENCE_JITTER_ON_SECONDS_DEFAULT);
    Preferences.initDouble(PREFERENCE_JITTER_OFF_SECONDS_KEY, PREFERENCE_JITTER_OFF_SECONDS_DEFAULT);

  }

  public Command feed(double spispeed, double indspeed) {
    m_indexer.set(indspeed);
    m_Spindexer.set(spispeed);
    System.out.println(indspeed);
    return null;
  }

  private static final double SPINDEXER_SPEED = .9;

  public Command runTilCancelledWithJitter() {
    return Commands.defer(() -> {
      double jitterOnTimeSeconds = Preferences.getDouble(PREFERENCE_JITTER_ON_SECONDS_KEY,
          PREFERENCE_JITTER_ON_SECONDS_DEFAULT);
      double jitterOffTimeSeconds = Preferences.getDouble(PREFERENCE_JITTER_OFF_SECONDS_KEY,
          PREFERENCE_JITTER_OFF_SECONDS_DEFAULT);

      return Commands.repeatingSequence(
          Commands.runOnce(() -> {
            // Speeds to set when both are spinning
            m_indexer.set(.8);
            m_Spindexer.set(SPINDEXER_SPEED);
          }),
          Commands.waitSeconds(jitterOnTimeSeconds),
          Commands.runOnce(() -> {
            // Speeds to set when only the spindexer (aka transfer) but not the indexer (aka
            // hopper) is spinning
            m_indexer.set(-.4);
            m_Spindexer.set(SPINDEXER_SPEED);
          }),
          Commands.waitSeconds(jitterOffTimeSeconds)).finallyDo(() -> {
            // What to do when the command ends -- aka the button is released
            m_indexer.set(0);
            m_Spindexer.set(0);
          });
    }, Set.of(this));
  }

  // public class Indexer extends Command {
  // // private Indexing_Subsystem m_indexer = new Indexing_Subsystem();
  // Timer timer;
  // Indexing_Subsystem indexer;
  // public int indexer_switch = 1;

  // public Indexer(Indexing_Subsystem subsystem) {

  // this.timer = new Timer();
  // this.indexer = subsystem;
  // }

  // @Override
  // public void initialize() {
  // timer.restart();
  // }

  // public void execute() {
  // // System.out.println(timer.get());

  // System.out.println("timer.get()");
  // if ((Math.round(timer.get()) % 2) == 0) {
  // indexer.feed(.5 * indexer_switch, .8 * indexer_switch);
  // } else {
  // indexer.feed(.8 * indexer_switch, 0);

  // }
  // }

  // }

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
