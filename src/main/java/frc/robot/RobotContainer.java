// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.AutoDriveToTower;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoWaitForReadyToShoot;
import frc.robot.commands.Shoot;
import frc.robot.commands.StopShoot;
import frc.robot.commands.climb.ClimbClimb;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.climb.ManualSpool;
import frc.robot.commands.indexer.ReverseSpindexer;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.RunRoller;
import frc.robot.commands.leds.ActiveAtFZoneLED;
import frc.robot.commands.leds.ActiveAtHubLED;
import frc.robot.commands.leds.BlinkLED;
import frc.robot.commands.leds.IdleLED;
import frc.robot.commands.leds.InactiveLED;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.turret.CalibrateTurret;
import frc.robot.commands.turret.ManualTurret;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.SwerveTelemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.io.ButtonBox;
import frc.robot.io.CommandButtonBox;
import frc.robot.subsystems.AuxSwerveSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.AutoDisplayUtil;
import frc.robot.utils.Logger;
import frc.robot.utils.MiscUtils;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotContainer {
    // singleton instance
    private static boolean hasIntialized = false;
    private static RobotContainer instance = null;

    public static synchronized RobotContainer getInstance() {
        if (!hasIntialized) {
            hasIntialized = true;
            instance = new RobotContainer();
        }

        return instance;
    }

    public final PowerDistribution pdp;

    public final CommandXboxController commandDriver1, commandDriver2;
    public final XboxController hidDriver1, hidDriver2;
    public final CommandButtonBox commandButtonBox;
    public final ButtonBox hidButtonBox;

    private SendableChooser<Command> autoChooser;

    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ClimbSubsystem climber;
    public final LEDSubsystem ledSub;
    public final LimelightSubsystem limelights;
    public final TurretSubsystem turret;
    public final ShooterSubsystem shooter;
    public final CommandSwerveDrivetrain swerve;
    public final SwerveTelemetry swerveTelem;
    public final AuxSwerveSubsystem auxSwerve;

    public final ActiveAtHubLED activeAtHubLEDCommand;
    public final ActiveAtFZoneLED activeAtFZoneLEDCommand;
    public final InactiveLED inactiveLEDCommand;
    public final IdleLED idleLEDCommand;
    public final BlinkLED autoLEDCommand;
    public final AimAtTarget aimAtHubCommand;
    public final AimAtTarget aimAtFZoneCommand;
    public final ReverseSpindexer reverseSpindexerCommand;
    //     public final LimelightCommand limelightCommand;

    private SwerveRequest.Idle swerveIdle;
    private SwerveRequest.ApplyRobotSpeeds swerveApplyRobotSpeeds;

    public final Field2d ntField;

    private final Debouncer readyToShootDebouncer;

    private RobotContainer() {
        pdp = new PowerDistribution(30, ModuleType.kRev);

        commandDriver1 = new CommandXboxController(0);
        hidDriver1 = commandDriver1.getHID();
        commandDriver2 = new CommandXboxController(1);
        hidDriver2 = commandDriver2.getHID();
        commandButtonBox = new CommandButtonBox(2);
        hidButtonBox = commandButtonBox.getHID();

        intake = new IntakeSubsystem();
        indexer = new IndexerSubsystem();
        climber = new ClimbSubsystem();
        ledSub = new LEDSubsystem();
        limelights = new LimelightSubsystem(
                LimelightConstants.Turret.cameraName,
                LimelightConstants.Side.cameraName,
                LimelightConstants.Turret.robotRelativePose,
                LimelightConstants.Side.robotRelativePose);
        // otherLimelight = new LimelightSubsystem(LimelightConstants.Side.cameraName,
        // LimelightConstants.Side.robotRelativePose);
        turret = new TurretSubsystem();
        shooter = new ShooterSubsystem();
        swerve = TunerConstants.createDrivetrain();
        swerveTelem = new SwerveTelemetry(SwerveConstants.maxTranslationVel.in(MetersPerSecond));
        auxSwerve = new AuxSwerveSubsystem();

        activeAtHubLEDCommand = new ActiveAtHubLED(ledSub, () -> isReadyToShoot());
        activeAtFZoneLEDCommand = new ActiveAtFZoneLED(ledSub);
        inactiveLEDCommand = new InactiveLED(ledSub);
        idleLEDCommand = new IdleLED(ledSub);
        autoLEDCommand = new BlinkLED(ledSub, LEDConstants.orange);
        aimAtHubCommand = AimAtTarget.atHub(turret, shooter, swerve);
        aimAtFZoneCommand = AimAtTarget.atFZone(turret, shooter, swerve);
        reverseSpindexerCommand = new ReverseSpindexer(indexer);

        ntField = new Field2d();
        SmartDashboard.putData("Field", ntField); // only ever call once

        readyToShootDebouncer = new Debouncer(0.4, DebounceType.kFalling);

        MiscUtils.changeSubsystemDefaultCommand(ledSub, idleLEDCommand, true);
        MiscUtils.changeSubsystemDefaultCommand(indexer, reverseSpindexerCommand, false);

        // limelightCommand =
        //         new LimelightCommand(turretLimelight, otherLimelight, swerve, () -> DriverStation.isEnabled());
        // CommandScheduler.getInstance().schedule(limelightCommand);

        addNamedCommands();
        setupSwerve();
        configureBindings();
        configureAutonomous();
    }

    private void setupSwerve() {
        swerveIdle = new SwerveRequest.Idle();

        swerveApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled()
                .whileTrue(swerve.applyRequest(() -> swerveIdle).ignoringDisable(true));
        swerve.registerTelemetry(swerveTelem::telemeterize);
    }

    private void configureBindings() {
        commandDriver1
                .b()
                .onTrue(new InstantCommand(() -> {
                            swerve.resetRotation(
                                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                            ? Rotation2d.kZero
                                            : Rotation2d.k180deg);
                            limelights.seedBothAbsolute(Angle.ofBaseUnits(
                                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 180,
                                    Degrees));
                        })
                        .ignoringDisable(true));

        commandDriver2
                .leftTrigger()
                .whileTrue(new RepeatCommand(new RunRoller(intake, false)))
                .whileTrue(new BlinkLED(ledSub, Color.kWhite));
        commandDriver2.leftBumper().whileTrue(new RepeatCommand(new ReverseIntake(intake)));
        commandDriver2.povUp().onTrue(new ClimbUp(climber, false));
        commandDriver2.povRight().or(commandDriver2.povLeft()).onTrue(new ClimbClimb(climber, false));
        commandDriver2.povDown().onTrue(new ClimbDown(climber, false));
        commandDriver2
                .rightTrigger()
                .whileTrue(new RepeatCommand(new ConditionalCommand(
                        new Shoot(indexer, intake, true),
                        new StopShoot(indexer, intake),
                        () -> isReadyToShoot()
                                && Robot.getInstance()
                                        .getTeleopLogic()
                                        .map((val) -> val.getIsHubActive().orElse(false))
                                        .orElse(true))))
                .onFalse(new StopShoot(indexer, intake));
        commandDriver2
                .x()
                .onTrue(new CalibrateTurret(turret).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        commandButtonBox
                .resetTurret()
                .onTrue(new CalibrateTurret(turret).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox.zeroClimb().onTrue(new InstantCommand(() -> climber.setAsZero()));
        commandButtonBox
                .deployIntake()
                .onTrue(new DeployIntake(intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox.seed().onTrue(new InstantCommand(limelights::seedSwerve));
        commandButtonBox
                .turretLeft()
                .whileTrue(new ManualTurret(turret, TurretConstants.Motor.manualSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .turretRight()
                .whileTrue(new ManualTurret(turret, -TurretConstants.Motor.manualSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .testShoot()
                // .onTrue(new ManualShoot(
                //                 shooter,
                //                 () -> ShooterConstants.maxManualSpeed.times((-hidButtonBox.getSliderAxis() + 1) /
                // 2.0),
                //                 true)
                //         .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                .onTrue(shooter.runSysId);
        commandButtonBox.alt().onTrue(new ManualShoot(shooter, () -> RPM.of(750), false));
        commandButtonBox
                .stopShoot()
                .onTrue(new StopShooter(shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox
                .spoolDown()
                .whileTrue(new ManualSpool(climber, -ClimberConstants.Motor.manualSpoolSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox
                .spoolUp()
                .whileTrue(new ManualSpool(climber, ClimberConstants.Motor.manualSpoolSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }

    private void addNamedCommands() {
        NamedCommands.registerCommand("ClimbClimb", new ClimbClimb(climber, true));
        NamedCommands.registerCommand("ClimbUp", new ClimbUp(climber, true));
        NamedCommands.registerCommand("AimAtHub", aimAtHubCommand);
        NamedCommands.registerCommand("Shoot", new AutoShoot(indexer, intake));
        NamedCommands.registerCommand("CalibrateTurret", new CalibrateTurret(turret));
        NamedCommands.registerCommand(
                "WaitForReadyToShoot", new AutoWaitForReadyToShoot(turret, Optional.of(Seconds.of(4))));
        NamedCommands.registerCommand("DeployIntake", new DeployIntake(intake));
        NamedCommands.registerCommand("Intake", new RunRoller(intake, false));
        NamedCommands.registerCommand(
                "DriveToTower", new AutoDriveToTower(swerve, climber, Optional.of(Seconds.of(5))));
    }

    public Command getAutonomousCommand() {
        Command auto;
        try {
            auto = autoChooser.getSelected();
        } catch (Exception e) {
            Logger.reportError(e);
            auto = null;
        }

        if (auto == null) {
            auto = Commands.print("No auto selected");
        }
        return auto;
    }

    private void configureAutonomous() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Logger.reportWarning("Failed to load pathplanner gui settings.json, using backup config", false);
            config = new RobotConfig(
                    Pounds.of(125), // 110 lbs + ~15 lbs for bumpers and battery
                    KilogramSquareMeters.of(6.883),
                    new ModuleConfig(
                            Meters.of(TunerConstants.FrontLeft.WheelRadius),
                            MetersPerSecond.of(TunerConstants.FrontLeft.SpeedAt12Volts),
                            1.2,
                            DCMotor.getKrakenX60(1),
                            Amps.of(60),
                            1),
                    swerve.getModuleLocations());
        }

        try {
            final Supplier<Pose2d> poseSupplier = () -> swerve.getState().Pose;
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier = () -> swerve.getState().Speeds;
            final BiConsumer<ChassisSpeeds, DriveFeedforwards> outputConsumer =
                    (speeds, ff) -> swerve.setControl(swerveApplyRobotSpeeds
                            .withSpeeds(ChassisSpeeds.discretize(speeds, 0.02))
                            .withWheelForceFeedforwardsX(ff.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(ff.robotRelativeForcesYNewtons()));
            final BooleanSupplier shouldFlipSupplier = () -> DriverStation.getAlliance()
                    .map((val) -> val == Alliance.Red)
                    .orElse(false);

            AutoBuilder.configure(
                    poseSupplier,
                    swerve::resetPose,
                    robotRelativeSpeedsSupplier,
                    outputConsumer,
                    new PPHolonomicDriveController(
                            SwerveConstants.pathplannerTranslationPID, SwerveConstants.pathplannerRotationPID),
                    config,
                    shouldFlipSupplier,
                    swerve);

            autoChooser = AutoBuilder.buildAutoChooser();
        } catch (Exception e) {
            Logger.reportError(e);
        }
        setupAutoDisplay();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void setupAutoDisplay() {
        // update the displayed auto path in SmartDashboard when ever the selection is changed
        // display is cleared in teleopInit
        if (autoChooser.getSelected() != null
                && !autoChooser.getSelected().getName().equals("InstantCommand")) {
            Logger.logString("", "selectedAuto", autoChooser.getSelected().getName());
        } else {
            Logger.logString("", "selectedAuto", "None");
        }

        autoChooser.onChange((selected) -> {
            if (autoChooser.getSelected() != null
                    && !autoChooser.getSelected().getName().equals("InstantCommand")) {
                Logger.logString("", "selectedAuto", autoChooser.getSelected().getName());
            } else {
                Logger.logString("", "selectedAuto", "None");
            }

            // if (DriverStation.isTeleopEnabled()) return;
            displayAuto();
        });

        /*
         * Robot.teleopInit clears the display
         * Robot.autonomousInit and Robot.disabledInit redraws the display
         */
    }

    public void displayAuto() {
        try {
            Command auto = autoChooser.getSelected();

            if (auto == null || auto.getName().equals("InstantCommand")) {
                AutoDisplayUtil.clearAutoPath();
                return;
            }
            AutoDisplayUtil.displayAutoPath(auto);
        } catch (Exception e) {
            Logger.reportError(e);
        }
    }

    public boolean isReadyToShoot() {
        return readyToShootDebouncer.calculate(shooter.isShooterAtTargetSpeed().orElse(false));
    }
}
