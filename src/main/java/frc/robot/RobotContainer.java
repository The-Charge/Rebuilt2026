// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {

    public final IndexerSubsystem indexer; // defines instance

    public RobotContainer() {
        configureBindings();
        indexer = new IndexerSubsystem(); // tells what instance is equal to
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
