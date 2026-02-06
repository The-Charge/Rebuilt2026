// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.indexer.SpinDownIndexer;
import frc.robot.commands.indexer.SpinUpIndexer;
import frc.robot.commands.intake.RunRoller;
import frc.robot.commands.leds.FriendlyZoneLED;
import frc.robot.commands.leds.IdleLED;
import frc.robot.commands.leds.NeutralZoneLED;
import frc.robot.commands.leds.OpposingZoneLED;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

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

    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ClimbSubsystem climber;
    public final LEDSubsystem ledSub;

    public final FriendlyZoneLED friendlyZoneLEDCommand;
    public final NeutralZoneLED neutralZoneLEDCommand;
    public final OpposingZoneLED opposingZoneLEDCommand;
    public final IdleLED idleLEDCommand;
    public final SpinUpIndexer spinUpIndexerCommand;
    public final SpinDownIndexer spinDownIndexerCommand;

    private RobotContainer() {
        pdp = new PowerDistribution();

        commandDriver1 = new CommandXboxController(0);
        hidDriver1 = commandDriver1.getHID();
        commandDriver2 = new CommandXboxController(1);
        hidDriver2 = commandDriver2.getHID();

        intake = new IntakeSubsystem();
        indexer = new IndexerSubsystem();
        climber = new ClimbSubsystem();
        ledSub = new LEDSubsystem();

        friendlyZoneLEDCommand = new FriendlyZoneLED(ledSub);
        neutralZoneLEDCommand = new NeutralZoneLED(ledSub);
        opposingZoneLEDCommand = new OpposingZoneLED(ledSub);
        idleLEDCommand = new IdleLED(ledSub);
        spinUpIndexerCommand = new SpinUpIndexer(indexer, false);
        spinDownIndexerCommand = new SpinDownIndexer(indexer);

        ledSub.setDefaultCommand(idleLEDCommand);

        configureBindings();
    }

    private void configureBindings() {
        commandDriver2.leftTrigger().whileTrue(new RunRoller(intake));
        commandDriver2.povUp().onTrue(new ClimbUp(climber, false));
        commandDriver2.povDown().onTrue(new ClimbDown(climber, false));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
