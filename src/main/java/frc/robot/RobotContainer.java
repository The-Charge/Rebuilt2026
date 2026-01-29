// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Indexer.SpinDownIndexer;
import frc.robot.commands.Indexer.SpinUpIndexer;
import frc.robot.commands.Indexer.StartGateIndexer;
import frc.robot.commands.Indexer.StopGateIndexer;
import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {

    // singleton instance
    private static RobotContainer instance = null;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();

        return instance;
    }

    public final PowerDistribution pdp;

    public final CommandXboxController commandDriver1, commandDriver2;
    public final XboxController hidDriver1, hidDriver2;

    public final IndexerSubsystem indexer; // defines instance

    private RobotContainer() {
        pdp = new PowerDistribution();

        commandDriver1 = new CommandXboxController(0);
        hidDriver1 = commandDriver1.getHID();
        commandDriver2 = new CommandXboxController(1);

        hidDriver2 = commandDriver2.getHID();

        indexer = new IndexerSubsystem(); // tells what instance is equal to

        configureBindings();
    }

    private void configureBindings() {
        commandDriver1.a().onTrue(new SpinUpIndexer(indexer));
        commandDriver1.b().onTrue(new InstantCommand(indexer::stop, indexer).ignoringDisable(true));
        commandDriver1.x().onTrue(new SpinDownIndexer(indexer));

        commandDriver2.a().onTrue(new StartGateIndexer(indexer));
        commandDriver2.b().onTrue(new InstantCommand(indexer::stop, indexer).ignoringDisable(true));
        commandDriver2.x().onTrue(new StopGateIndexer(indexer));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
