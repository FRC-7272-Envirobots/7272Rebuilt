package frc.robot.commands;

import java.util.Optional;


import java.awt.Color;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexing_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.Shooter_Subsystem;

public class Routine {

    private Intake_Subsystem intake;
    private Shooter_Subsystem shooter;
    private Indexing_Subsystem indexer;
    private Lightstrip lightstrip;

    //  public Command intakeRoutine() {
    //     return intake.runIntake()
    //             .alongWith(lightstrip.setColorCommand(Color.RED))
    //             .until(intakeSensor::isNoteCaptured)
    //             .andThen(lightstrip.flashColor(Color.WHITE, 0.1, 5.0));
    // }

    // public Command shootRoutine(double speed) {
    //     return Commands.parallel(
    //             shooter.runShooter(speed, Optional.empty()),
    //             Commands.sequence(
    //                             Commands.waitSeconds(.2),
    //                             intake.runIntake(),
    //                             new StartEndCommand(
    //                                             () -> {
    //                                                     lightstrip.setShootCompletedColor();
    //                                             },
    //                                             () -> {
    //                                             }),
    //                             new WaitCommand(5)));
    public Command ArmDown(){
        return intake.armmove(-0.2)
                .alongWith(lightstrip.setColorCommand(Color.YELLOW))
                .andThen(Commands.waitSeconds(0.2))
                .andThen(intake.armmove(0));
        
    }
    // public Command ArmUp(){
    //     return Commands.sequence(intake.armmove(0.2)
    //             .alongWith(lightstrip.setColorCommand(Color.YELLOW)),
    //             Commands.waitSeconds(0.2)
    //             .andThen(intake.armmove(0))
    //     );
    // }
    // public Command ShootCommand(double shooterspeed){
    //     return Commands.sequence(
    //         new RunCommand(()->shooter, null)
    //        // .alongWith(lightstrip.flashColor(Color.ORANGE,0.2,0.3)),
    //         Commands.waitSeconds(0.3),
    //         // .alongWith(lightstrip.flashColor(Color.RED,0.2,20)),
    //         indexer.feed(0.8,0.5)


    //     );

    // }
    
}
