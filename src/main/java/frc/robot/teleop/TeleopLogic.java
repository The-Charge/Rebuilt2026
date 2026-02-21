package frc.robot.teleop;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.leds.DualBlinkLED;
import frc.robot.commands.leds.RainbowLED;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.AutoDisplayUtil;
import frc.robot.utils.ControllerUtil;
import frc.robot.utils.Logger;
import frc.robot.utils.MiscUtils;
import java.util.Optional;

public class TeleopLogic {

    private static final String subsystemName = "Teleop";

    private Optional<Pose2d> lastRobotPose;
    private Timer teleopTimer;
    private Optional<Alliance> autoWinner;
    private Optional<Time> lastTimeLeftInPhase;
    private Optional<TeleopPhase> lastPhase;
    private Optional<Boolean> lastHubActive;
    private Optional<TeleopAssistMode> lastAssistMode;

    public TeleopLogic() {}

    public void startTeleop() {
        teleopTimer = new Timer();
        teleopTimer.restart();

        autoWinner = Optional.empty(); // default until teleopPerodic assigns it a value
        lastRobotPose = Optional.empty();
        lastTimeLeftInPhase = Optional.empty();
        lastPhase = Optional.empty();
        lastHubActive = Optional.empty();
        lastAssistMode = Optional.empty();

        CommandScheduler.getInstance().schedule(new DeployIntake(RobotContainer.getInstance().intake));
        AutoDisplayUtil.clearAutoPath();
    }

    public void teleopPeriodic() {
        // get teleop phase and phase time
        TeleopPhase phase = TeleopPhase.fromTeleopTimer(Seconds.of(teleopTimer.get()));
        Time timeLeftInPhase = TeleopPhase.getTimeLeftInPhase(Seconds.of(teleopTimer.get()));

        // get which alliance won auto
        if (autoWinner == null || autoWinner.isEmpty()) {
            // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
            String gameMessage = DriverStation.getGameSpecificMessage();
            Logger.logString(subsystemName, "gameSpecificMessage", gameMessage);

            if (gameMessage != null && !gameMessage.isEmpty()) {
                if (gameMessage.charAt(0) == 'R') {
                    autoWinner = Optional.of(Alliance.Red);
                } else if (gameMessage.charAt(0) == 'B') {
                    autoWinner = Optional.of(Alliance.Blue);
                }
            }
        }

        // get robot pose
        Pose2d robotPose = RobotContainer.getInstance().swerve.getPosition();

        // get zone and alliance info
        FieldZone zone = FieldZone.fromRobotPose(robotPose);
        Optional<FieldZone> lastZone = lastRobotPose.map((val) -> FieldZone.fromRobotPose(val));
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        boolean isInFriendlyZone = zone == FieldZone.BLUE && alliance == Alliance.Blue
                || zone == FieldZone.RED && alliance == Alliance.Red;
        boolean isInNeutralZone = zone == FieldZone.NEUTRAL;
        boolean isInEnemyZone = !isInFriendlyZone && !isInNeutralZone;

        // check if we have reached the final 3 seconds of the phase
        double prevSecsLeft = lastTimeLeftInPhase.map((val) -> val.in(Seconds)).orElse(140.0);
        boolean hasReachedPrePhaseChange =
                timeLeftInPhase.in(Seconds) <= 3 && prevSecsLeft > 3 && phase != TeleopPhase.ENDGAME;

        // check if our hub is active
        boolean isHubActive;
        if (autoWinner.isPresent()) {
            isHubActive = isFriendlyHubActive(alliance, autoWinner.get(), phase);
        } else {
            isHubActive = true;
        }

        // check if hub status has changed
        boolean hasHubStateChanged = lastHubActive.isEmpty() || isHubActive != lastHubActive.get();

        // check if the field zone has changed
        boolean hasZoneChanged = lastZone.isEmpty() || lastZone.get() != zone;

        // check if the teleop phase has changed
        boolean hasPhaseChanged = lastPhase.isEmpty() || phase != lastPhase.get();

        // determine what assist mode the robot should be in
        // fallback behavior is to be ready to shoot so that if the teleop logic bugs out, the drivers should still be
        // able to shoot
        TeleopAssistMode assistMode = TeleopAssistMode.ACTIVE_TURRET_AT_HUB;
        if (isInFriendlyZone && isHubActive) {
            assistMode = TeleopAssistMode.ACTIVE_TURRET_AT_HUB;
        } else if (isInEnemyZone || isInFriendlyZone && !isHubActive) {
            assistMode = TeleopAssistMode.INACTIVE_TURRET;
        } else if (isInNeutralZone) {
            assistMode = TeleopAssistMode.ACTIVE_TURRET_AT_FRIENDLY_ZONE;
        }

        // handle mode changes, it's just a state machine
        boolean assistModeChanged = lastAssistMode.isEmpty() || lastAssistMode.get() != assistMode;
        if (assistModeChanged) {
            switch (assistMode) {
                case ACTIVE_TURRET_AT_HUB: {
                    enterActiveAtHubMode();
                    break;
                }
                case ACTIVE_TURRET_AT_FRIENDLY_ZONE: {
                    enterActiveAtFZoneMode();
                    break;
                }
                case INACTIVE_TURRET: {
                    enterInactiveMode();
                    break;
                }
            }
        }

        // schedule timed events
        if (hasReachedPrePhaseChange) {
            CommandScheduler.getInstance()
                    .schedule(new DualBlinkLED(
                            RobotContainer.getInstance().ledSub, Color.kBlue, Color.kRed, timeLeftInPhase));

            final double rumbleStrength = 0.75;
            CommandScheduler.getInstance()
                    .schedule(new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                ControllerUtil.scheduleControllerRumble(0, rumbleStrength, rumbleStrength, 0.25);
                                ControllerUtil.scheduleControllerRumble(1, rumbleStrength, rumbleStrength, 0.25);
                            }),
                            new WaitCommand(0.35),
                            new InstantCommand(() -> {
                                ControllerUtil.scheduleControllerRumble(0, rumbleStrength, rumbleStrength, 0.25);
                                ControllerUtil.scheduleControllerRumble(1, rumbleStrength, rumbleStrength, 0.25);
                            })));
        }
        if (hasPhaseChanged && phase == TeleopPhase.ENDGAME) {
            CommandScheduler.getInstance().schedule(new RainbowLED(RobotContainer.getInstance().ledSub, Seconds.of(5)));

            final double rumbleStrength = 0.75;
            ControllerUtil.scheduleControllerRumble(0, rumbleStrength, rumbleStrength, 1);
            ControllerUtil.scheduleControllerRumble(1, rumbleStrength, rumbleStrength, 1);
        }

        // log debug data
        Logger.logString(subsystemName, "phase", phase.toString());
        Logger.logDouble(subsystemName, "secsLeftInPhase", timeLeftInPhase.in(Seconds));
        Logger.logBool(subsystemName, "isFriendlyHubActive", isHubActive);
        Logger.logString(subsystemName, "assistMode", assistMode.toString());
        Logger.logString(
                subsystemName,
                "autoWinningAlliance",
                autoWinner.map((val) -> val.toString()).orElse("None"));
        Logger.logString(subsystemName, "zone", zone.toString());

        lastRobotPose = Optional.of(robotPose);
        lastTimeLeftInPhase = Optional.of(timeLeftInPhase);
        lastPhase = Optional.of(phase);
        lastHubActive = Optional.of(isHubActive);
        lastAssistMode = Optional.of(assistMode);
    }

    public void endTeleop() {}

    private void enterActiveAtHubMode() {
        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().activeAtHubLEDCommand, false);

        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().turret, RobotContainer.getInstance().pointAtHubCommand, true);
        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().shooter, RobotContainer.getInstance().prepShootAtHubCommand, true);
    }

    private void enterActiveAtFZoneMode() {
        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().activeAtFZoneLEDCommand, false);

        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().turret, RobotContainer.getInstance().pointAtFZoneCommand, true);
        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().shooter, RobotContainer.getInstance().prepShootAtFZoneCommand, true);
    }

    private void enterInactiveMode() {
        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().inactiveLEDCommand, false);

        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().turret, RobotContainer.getInstance().centerTurretCommand, true);

        RobotContainer.getInstance().shooter.removeDefaultCommand();
        CommandScheduler.getInstance().schedule(new StopShooter(RobotContainer.getInstance().shooter));
    }

    private boolean isFriendlyHubActive(Alliance alliance, Alliance autoWinner, TeleopPhase phase) {
        boolean isWinningAlliance = alliance == autoWinner;

        switch (phase) {
            case TRANSITION_SHIFT:
            case ENDGAME: {
                return true;
            }
            case SHIFT1:
            case SHIFT3: {
                return !isWinningAlliance;
            }
            case SHIFT2:
            case SHIFT4: {
                return isWinningAlliance;
            }
            default: {
                return true;
            }
        }
    }

    public static Translation2d getFriendlyZoneTarget(Translation2d robotPose) {
        boolean isInTopHalf = robotPose.getMeasureY().gt(FieldConstants.fieldYCenter);
        boolean isBlue =
                DriverStation.getAlliance().map((val) -> val == Alliance.Blue).orElse(true);

        if (isBlue) {
            if (isInTopHalf) {
                return FieldConstants.blueZoneTopLoc;
            } else {
                return FieldConstants.blueZoneBottomLoc;
            }
        } else {
            if (isInTopHalf) {
                return FieldConstants.redZoneTopLoc;
            } else {
                return FieldConstants.redZoneBottomLoc;
            }
        }
    }
}
