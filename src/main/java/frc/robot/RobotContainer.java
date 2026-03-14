// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoPrepShootAtHub;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoStopShoot;
import frc.robot.commands.WaitForReadyToShoot;
import frc.robot.commands.climb.ClimbClimb;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.climb.ManualSpool;
import frc.robot.commands.indexer.SpinDownIndexer;
import frc.robot.commands.indexer.SpinUpIndexer;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.RunRoller;
import frc.robot.commands.intake.StopRoller;
import frc.robot.commands.leds.ActiveAtFZoneLED;
import frc.robot.commands.leds.ActiveAtHubLED;
import frc.robot.commands.leds.BlinkLED;
import frc.robot.commands.leds.IdleLED;
import frc.robot.commands.leds.InactiveLED;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.PrepShootAtHub;
import frc.robot.commands.shooter.PrepShootAtPoint;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.turret.AlignTurret;
import frc.robot.commands.turret.CalibrateTurret;
import frc.robot.commands.turret.CenterTurret;
import frc.robot.commands.turret.ManualTurret;
import frc.robot.commands.vision.LimelightCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.io.ButtonBox;
import frc.robot.io.CommandButtonBox;
import frc.robot.subsystems.AuxSwerveSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.teleop.TeleopLogic;
import frc.robot.utils.ControllerUtil;
import frc.robot.utils.Logger;
import frc.robot.utils.MiscUtils;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
    public final CommandButtonBox commandButtonBox;
    public final ButtonBox hidButtonBox;

    private SendableChooser<Command> autoChooser;

    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ClimbSubsystem climber;
    public final LEDSubsystem ledSub;
    public final LimelightSubsystem turretLimelight;
    public final LimelightSubsystem otherLimelight;
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
    public final AlignTurret pointAtHubCommand;
    public final CenterTurret centerTurretCommand;
    public final PrepShootAtPoint prepShootAtFZoneCommand;
    public final PrepShootAtHub prepShootAtHubCommand;
    public final AlignTurret pointAtFZoneCommand;
    public final LimelightCommand limelightCommand;

    private SwerveRequest.FieldCentric swerveFieldCentricDrive;
    private SwerveRequest.RobotCentric swerveRobotCentricDrive;
    private SwerveRequest.FieldCentricFacingAngle swerveFieldCentricFacingAngleDrive;
    private SwerveRequest.SwerveDriveBrake swerveBrake;
    private SwerveRequest.Idle swerveIdle;
    private SwerveRequest.ApplyRobotSpeeds swerveApplyRobotSpeeds;
    public Command swerveFieldCentricDriveCommand;
    public Command swerveRobotCentricDriveCommand;
    public Command swerveFieldCentricFacingAngleDriveCommand;
    private Optional<Rotation2d> lastDefinedRotation = Optional.empty();

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
        turretLimelight = new LimelightSubsystem("turret", LimelightConstants.turretPose);
        otherLimelight = new LimelightSubsystem("other", LimelightConstants.otherPose);
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
        pointAtHubCommand = AlignTurret.atHub(turret, swerve, otherLimelight, () -> DriverStation.getAlliance()
                .orElse(Alliance.Blue));
        centerTurretCommand = new CenterTurret(turret);
        prepShootAtFZoneCommand = new PrepShootAtPoint(
                shooter,
                () -> TeleopLogic.getFriendlyZoneTarget(
                        swerve.getStateCopy().Pose.getTranslation()));
        prepShootAtHubCommand = new PrepShootAtHub(shooter, () -> DriverStation.getAlliance());
        pointAtFZoneCommand = AlignTurret.atPoint(
                turret,
                swerve,
                otherLimelight,
                () -> TeleopLogic.getFriendlyZoneTarget(
                        swerve.getStateCopy().Pose.getTranslation()));

        MiscUtils.changeSubsystemDefaultCommand(ledSub, idleLEDCommand, true);

        limelightCommand =
                new LimelightCommand(turretLimelight, otherLimelight, swerve, () -> DriverStation.isEnabled());
        CommandScheduler.getInstance().schedule(limelightCommand);

        addNamedCommands();
        setupSwerve();
        configureBindings();
        configureAutonomous();

        swerve.registerTelemetry(swerveTelem::telemeterize);
    }

    private void setupSwerve() {
        swerveFieldCentricDrive = new SwerveRequest.FieldCentric()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        swerveRobotCentricDrive = new SwerveRequest.RobotCentric()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        swerveFieldCentricFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        swerveBrake = new SwerveRequest.SwerveDriveBrake();
        swerveIdle = new SwerveRequest.Idle();
        swerveApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        DoubleSupplier speedShifter = () -> hidDriver1.getRightTriggerAxis() >= 0.5 ? 0.25 : 1;
        DoubleSupplier intakeMultiplier = () -> hidDriver2.getLeftTriggerAxis() >= 0.5 ? 0.5 : 1;

        swerveFieldCentricDriveCommand = swerve.applyRequest(() -> swerveFieldCentricDrive
                .withVelocityX(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftY(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()
                        * intakeMultiplier.getAsDouble()))
                .withVelocityY(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftX(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()
                        * intakeMultiplier.getAsDouble()))
                .withRotationalRate(SwerveConstants.maxAngularVel.times(
                        ControllerUtil.applyLinearDeadband(-hidDriver1.getRightX(), SwerveConstants.joystickDeadband)
                                * speedShifter.getAsDouble())));

        swerveRobotCentricDriveCommand = swerve.applyRequest(() -> swerveRobotCentricDrive
                .withVelocityX(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftY(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()
                        * intakeMultiplier.getAsDouble()))
                .withVelocityY(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftX(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()
                        * intakeMultiplier.getAsDouble()))
                .withRotationalRate(SwerveConstants.maxAngularVel.times(
                        ControllerUtil.applyLinearDeadband(-hidDriver1.getRightX(), SwerveConstants.joystickDeadband)
                                * speedShifter.getAsDouble())));

        swerveFieldCentricFacingAngleDriveCommand = swerve.applyRequest(() -> {
            ChassisSpeeds speed = swerve.getStateCopy().Speeds;
            if (Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) > 0.01) {
                lastDefinedRotation =
                        Optional.of(new Rotation2d(Math.atan2(speed.vyMetersPerSecond, speed.vxMetersPerSecond)));
            }

            var req = swerveFieldCentricFacingAngleDrive
                    .withVelocityX(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                    -hidDriver1.getLeftY(),
                                    SwerveConstants.joystickDeadband,
                                    SwerveConstants.joystickExponent)
                            * speedShifter.getAsDouble()
                            * intakeMultiplier.getAsDouble()))
                    .withVelocityY(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                    -hidDriver1.getLeftX(),
                                    SwerveConstants.joystickDeadband,
                                    SwerveConstants.joystickExponent)
                            * speedShifter.getAsDouble()
                            * intakeMultiplier.getAsDouble()));
            if (lastDefinedRotation.isPresent()) {
                req = req.withTargetDirection(lastDefinedRotation.get());
            }

            return req;
        });

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(swerveFieldCentricDriveCommand);
        commandDriver1
                .b()
                .onTrue(new InstantCommand(() -> {
                            swerve.resetRotation(
                                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                            ? Rotation2d.kZero
                                            : Rotation2d.k180deg);
                            // limelightCommand.seedFromAbsolute(
                            //         DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 180);
                        })
                        .ignoringDisable(true));
        commandDriver1.x().whileTrue(swerve.applyRequest(() -> swerveBrake));
        commandDriver1
                .leftTrigger()
                .onTrue(new InstantCommand(() ->
                                MiscUtils.changeSubsystemDefaultCommand(swerve, swerveRobotCentricDriveCommand, false))
                        .ignoringDisable(true))
                .onFalse(new InstantCommand(() ->
                                MiscUtils.changeSubsystemDefaultCommand(swerve, swerveFieldCentricDriveCommand, false))
                        .ignoringDisable(true));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled()
                .whileTrue(swerve.applyRequest(() -> swerveIdle).ignoringDisable(true))
                .onTrue(new InstantCommand(() -> {
                            lastDefinedRotation = Optional.empty();
                        })
                        .ignoringDisable(true)); // reset snake mode
    }

    private void configureBindings() {
        // TODO: make swerve turn so that intake automatically faces the direction of travel while the intake is running
        commandDriver2
                .leftTrigger()
                .whileTrue(new RunRoller(intake, false))
                .whileTrue(new BlinkLED(ledSub, Color.kWhite));
        commandDriver2.povUp().onTrue(new ClimbUp(climber, false));
        commandDriver2.povRight().or(commandDriver2.povLeft()).onTrue(new ClimbClimb(climber, false));
        commandDriver2.povDown().onTrue(new ClimbDown(climber, false));
        commandDriver2
                .rightTrigger()
                .whileTrue(new RepeatCommand(new ConditionalCommand(
                        new SpinUpIndexer(indexer), new SpinDownIndexer(indexer), () -> isReadyToShoot())))
                .onFalse(new SpinDownIndexer(indexer));
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
        commandButtonBox.seed().onTrue(new InstantCommand(limelightCommand::seedFromIMU));
        commandButtonBox
                .turretLeft()
                .whileTrue(new ManualTurret(turret, TurretConstants.manualSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .turretRight()
                .whileTrue(new ManualTurret(turret, -TurretConstants.manualSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .testShoot()
                .onTrue(new ManualShoot(
                                shooter,
                                () -> ShooterConstants.ShootConfig.maxManualSpeed.times(
                                        (-hidButtonBox.getSliderAxis() + 1) / 2.0))
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .stopShoot()
                .onTrue(new StopShooter(shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox
                .spoolDown()
                .whileTrue(new ManualSpool(climber, -ClimberConstants.manualSpoolSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox
                .spoolUp()
                .whileTrue(new ManualSpool(climber, ClimberConstants.manualSpoolSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }

    private void addNamedCommands() {
        NamedCommands.registerCommand("ClimbClimb", new ClimbClimb(climber, true));
        NamedCommands.registerCommand("ClimbUp", new ClimbUp(climber, true));
        NamedCommands.registerCommand(
                "PrepShootAtHub", new AutoPrepShootAtHub(shooter, turret, () -> DriverStation.getAlliance()
                        .orElse(Alliance.Blue)));
        NamedCommands.registerCommand("Shoot", new AutoShoot(indexer, intake));
        NamedCommands.registerCommand("StopShoot", new AutoStopShoot(shooter, indexer, intake));
        NamedCommands.registerCommand("CalibrateTurret", new CalibrateTurret(turret));
        NamedCommands.registerCommand("WaitForReadyToShoot", new WaitForReadyToShoot(Optional.of(Seconds.of(4))));
        NamedCommands.registerCommand("DeployIntake", new DeployIntake(intake));
        NamedCommands.registerCommand("Intake", new RunRoller(intake, true));
        NamedCommands.registerCommand("StopIntake", new StopRoller(intake));
    }

    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();
        if (auto == null) {
            auto = Commands.print("No auto selected");
        }

        return auto;
    }

    private void configureAutonomous() {
        // TODO: add pathplanner configs to swerve
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
        // update the displayed auto path in smartdashboard when ever the selection is changed
        // display is cleared in teleopInit
        // if (autoChooser.getSelected() != null
        //         && !autoChooser.getSelected().getName().equals("InstantCommand")) {
        //     Logger.logString("", "selectedAuto", autoChooser.getSelected().getName());
        // } else {
        //     Logger.logString("", "selectedAuto", "None");
        // }

        // autoChooser.onChange((selected) -> {
        //     if (DriverStation.isTeleopEnabled()) return;

        //     displayAuto();

        //     if (autoChooser.getSelected() != null
        //             && !autoChooser.getSelected().getName().equals("InstantCommand")) {
        //         Logger.logString("", "selectedAuto", autoChooser.getSelected().getName());
        //     } else {
        //         Logger.logString("", "selectedAuto", "None");
        //     }
        // });

        /*
         * Robot.teleopInit clears the display
         * Robot.autonomousInit redraws the display
         */
    }

    public void displayAuto() {
        try {
            Command auto = autoChooser.getSelected();

            if (auto == null || auto.getName().equals("InstantCommand")) {
                // AutoDisplayUtil.clearAutoPath();
                return;
            }

            boolean isRed = DriverStation.getAlliance()
                    .map((val) -> val == Alliance.Red)
                    .orElse(false);
            // AutoDisplayUtil.displayAutoPath(auto, isRed);
        } catch (Exception e) {
            Logger.reportError(e);
        }
    }

    public boolean isReadyToShoot() {
        return shooter.isShooterAtTargetSpeed().orElse(false);
    }
}
