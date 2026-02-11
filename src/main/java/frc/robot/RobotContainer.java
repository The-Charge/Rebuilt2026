// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.indexer.SpinDownIndexer;
import frc.robot.commands.indexer.SpinUpIndexer;
import frc.robot.commands.intake.RunRoller;
import frc.robot.commands.leds.BlinkLED;
import frc.robot.commands.leds.FriendlyZoneLED;
import frc.robot.commands.leds.IdleLED;
import frc.robot.commands.leds.NeutralZoneLED;
import frc.robot.commands.leds.OpposingZoneLED;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.AutoDisplayHelper;
import frc.robot.utils.Logger;
import java.util.Optional;

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

    private SendableChooser<Command> autoChooser;

    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ClimbSubsystem climber;
    public final LEDSubsystem ledSub;

    public final FriendlyZoneLED activeFriendlyZoneLEDCommand;
    public final FriendlyZoneLED inactiveFriendlyZoneLEDCommand;
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

        activeFriendlyZoneLEDCommand = new FriendlyZoneLED(ledSub, true);
        inactiveFriendlyZoneLEDCommand = new FriendlyZoneLED(ledSub, false);
        neutralZoneLEDCommand = new NeutralZoneLED(ledSub);
        opposingZoneLEDCommand = new OpposingZoneLED(ledSub);
        idleLEDCommand = new IdleLED(ledSub);
        spinUpIndexerCommand = new SpinUpIndexer(indexer, false);
        spinDownIndexerCommand = new SpinDownIndexer(indexer);

        ledSub.setDefaultCommand(idleLEDCommand);

        configureBindings();
        configureAutonomous();
        addNamedCommands();
    }

    private void configureBindings() {
        // TODO: make swerve turn so that intake automatically faces the direction of travel while the intake is running
        commandDriver2
                .leftTrigger()
                .whileTrue(new ParallelCommandGroup(
                        new RunRoller(intake), new ScheduleCommand(new BlinkLED(ledSub, Color.kWhite))));
        commandDriver2.povUp().onTrue(new ClimbUp(climber, false));
        commandDriver2.povDown().onTrue(new ClimbDown(climber, false));
    }

    private void addNamedCommands() {
        NamedCommands.registerCommand("ClimbUp", new ClimbUp(climber, true));
        NamedCommands.registerCommand("ClimbDown", new ClimbDown(climber, true));
        NamedCommands.registerCommand("PrepShooter", Commands.print("PrepShooter Command not implemented"));
        NamedCommands.registerCommand("Shoot", Commands.print("Shoot Command not implemented"));
        NamedCommands.registerCommand("HaltShooter", Commands.print("HaltShooter COmmand not implemented"));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private void configureAutonomous() {
        // TODO: add pathplanner configs to swerve
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Logger.reportWarning("Failed to load pathplanner gui settings.json", false);
            config = new RobotConfig(
                    Pounds.of(115),
                    KilogramSquareMeters.of(3),
                    new ModuleConfig(Inches.of(2), FeetPerSecond.of(14), 0, null, Amps.of(60), 4));
        }

        AutoBuilder.configure(
                () -> new Pose2d(),
                pose -> {},
                () -> new ChassisSpeeds(),
                (ChassisSpeeds cs, DriveFeedforwards dff) -> {},
                new PPHolonomicDriveController(new PIDConstants(0), new PIDConstants(0)),
                config,
                () -> false); // fix this with real swerve stuff

        autoChooser = AutoBuilder.buildAutoChooser();
        setupAutoDisplay();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Field", new Field2d());
    }

    private void setupAutoDisplay() {
        // update the displayed auto path in smartdashboard when ever the selection is changed
        // display is cleared in teleopInit
        if (autoChooser.getSelected() != null) {
        }
        // LoggingManager.logValue("SelectedAuto", autoChooser.getSelected().getName());
        else {
        }
        // LoggingManager.logValue("SelectedAuto", "Null");

        autoChooser.onChange((selected) -> {
            if (DriverStation.isTeleopEnabled()) // don't display auton path in teleop
            return;

            displayAuto();

            if (autoChooser.getSelected() != null) {
            }
            // LoggingManager.logValue("SelectedAuto", autoChooser.getSelected().getName());
            else {
            }
            // LoggingManager.logValue("SelectedAuto", "Null");
        });

        /*
         * Robot.teleopInit clears the display
         * Robot.autonomousInit redraws the display
         */
    }

    public void displayAuto() {
        Command auto = autoChooser.getSelected();
        System.out.println(auto.getName());

        if (auto.getName().equals("InstantCommand")) {
            AutoDisplayHelper.clearAutoPath();
            return;
        }

        boolean isRed = false;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) isRed = true;

        AutoDisplayHelper.displayAutoPath(auto, isRed);
    }
}
