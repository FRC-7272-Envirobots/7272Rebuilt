package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Indexer extends Command {
  // private Indexing_Subsystem m_indexer = new Indexing_Subsystem();
  Timer timer;
  Indexing_Subsystem indexer;
  public int indexer_switch = 1;

  public Indexer(Indexing_Subsystem subsystem) {

    this.timer = new Timer();
    this.indexer = subsystem;
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  public void execute() {
    // System.out.println(timer.get());

    System.out.println("timer.get()");
    if ((Math.round(timer.get()) % 2) == 0) {
      indexer.feed(.5 * indexer_switch, .8 * indexer_switch);
    } else {
      indexer.feed(.8 * indexer_switch, 0);

    }
  }

}
