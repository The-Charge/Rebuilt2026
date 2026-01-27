// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Indexer.SpinDownIndexer;
import frc.robot.commands.Indexer.SpinUpIndexer;
import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {

    public final IndexerSubsystem indexer; // defines instance

    public final CommandXboxController command1;

    public RobotContainer() {
        indexer = new IndexerSubsystem(); // tells what instance is equal to

        command1 = new CommandXboxController(0);

        configureBindings();
    }

    private void configureBindings() {
        command1.a().onTrue(new SpinUpIndexer(indexer));
        command1.b().onTrue(new InstantCommand(indexer::stop, indexer).ignoringDisable(true));
        command1.x().onTrue(new SpinDownIndexer(indexer));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
