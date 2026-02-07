// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.leds.BlinkLED;
import frc.robot.commands.leds.DualBlinkLED;
import frc.robot.commands.leds.RainbowLED;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.ControllerUtil;
import frc.robot.utils.FieldZone;
import frc.robot.utils.Logger;
import frc.robot.utils.TeleopPhase;
import java.util.Optional;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    // teleop parameters
    private Optional<Pose2d> lastRobotPose;
    private Optional<Timer> teleopTimer;
    private Optional<Alliance> autoWinner;
    private Optional<Time> lastTimeLeftInPhase;
    private Optional<TeleopPhase> lastTeleopPhase;
    private Optional<Boolean> lastHubActive;

    public Robot() {
        Logger.init(); // DO NOT DELETE ; start logger
        RobotContainer.getInstance(); // DO NOT DELETE ; create singleton instance

        CANMonitor.setConnectionChangeCallback((id, connected) -> {
            if (connected == true) {
                Logger.println(String.format("Reconnected to CAN device %d", id));
                return;
            }

            Logger.reportWarning(String.format("Lost connection to CAN device %d", id), false);
            CommandScheduler.getInstance()
                    .schedule(new BlinkLED(RobotContainer.getInstance().ledSub, Color.kRed, Seconds.of(2)));
        });

        lastRobotPose = Optional.empty();
        teleopTimer = Optional.empty();
        autoWinner = Optional.empty();
        lastTimeLeftInPhase = Optional.empty();
        lastTeleopPhase = Optional.empty();
        lastHubActive = Optional.empty();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        ControllerUtil.periodic(RobotContainer.getInstance().hidDriver1, RobotContainer.getInstance().hidDriver2);

        Logger.logPDP(RobotContainer.getInstance().pdp);
        CANMonitor.logCANDeviceStatus(
                "PDP",
                RobotContainer.getInstance().pdp.getModule(),
                CANMonitor.isPDPConnected(RobotContainer.getInstance().pdp));
        Alerts.pdpDisconnected.set(!CANMonitor.isPDPConnected(RobotContainer.getInstance().pdp));

        Alerts.driver1Missing.set(!RobotContainer.getInstance().hidDriver1.isConnected());
        Alerts.driver2Missing.set(!RobotContainer.getInstance().hidDriver2.isConnected());
        Alerts.fmsConnected.set(DriverStation.isFMSAttached());

        double batteryVoltage = RobotContainer.getInstance().pdp.getVoltage();
        if (batteryVoltage <= 10) {
            Alerts.lowBattery.set(false);
            Alerts.criticalBattery.set(true);
        } else if (batteryVoltage <= 11) {
            Alerts.lowBattery.set(true);
            Alerts.criticalBattery.set(false);
        } else {
            Alerts.lowBattery.set(false);
            Alerts.criticalBattery.set(false);
        }

        Logger.logString(
                "",
                "autoWinningAlliance",
                autoWinner.map((val) -> val.toString()).orElse("None"));
    }

    @Override
    public void disabledInit() {
        RobotContainer.getInstance().indexer.stopAll();
        RobotContainer.getInstance().intake.stopRoller();
        RobotContainer.getInstance().climber.stopAll();

        RobotContainer.getInstance().ledSub.setDefaultCommand(RobotContainer.getInstance().idleLEDCommand);

        ControllerUtil.cancelControllerRumbles(0);
        ControllerUtil.cancelControllerRumbles(1);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();

        RobotContainer.getInstance().ledSub.setDefaultCommand(RobotContainer.getInstance().idleLEDCommand);

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        teleopTimer = Optional.of(new Timer());
        teleopTimer.get().start();
        autoWinner = Optional.empty(); // default until teleopPerodic assigns it a value
        lastRobotPose = Optional.empty();
        lastTimeLeftInPhase = Optional.empty();
        lastTeleopPhase = Optional.empty();
        lastHubActive = Optional.empty();

        CommandScheduler.getInstance().schedule(new DeployIntake(RobotContainer.getInstance().intake));
    }

    @Override
    public void teleopPeriodic() {
        // get teleop phase and phase time
        Optional<TeleopPhase> teleopPhase;
        Optional<Time> timeLeftInPhase;
        if (teleopTimer != null && teleopTimer.isPresent()) {
            teleopPhase = Optional.of(
                    TeleopPhase.fromTeleopTimer(Seconds.of(teleopTimer.get().get())));
            timeLeftInPhase = Optional.of(
                    TeleopPhase.getTimeLeftInPhase(Seconds.of(teleopTimer.get().get())));
        } else {
            teleopPhase = Optional.empty();
            timeLeftInPhase = Optional.empty();
        }

        // get which alliance won auto
        if (autoWinner == null || autoWinner.isEmpty()) {
            // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
            String gameMessage = DriverStation.getGameSpecificMessage();
            if (gameMessage != null && !gameMessage.isEmpty()) {
                if (gameMessage.charAt(0) == 'R') {
                    autoWinner = Optional.of(Alliance.Red);
                } else if (gameMessage.charAt(0) == 'B') {
                    autoWinner = Optional.of(Alliance.Blue);
                }
            }
        }

        // get robot pose
        Pose2d robotPose = Pose2d.kZero; // TODO: replace with value from odo

        // get zone and alliance info
        FieldZone currentZone = FieldZone.fromRobotPose(robotPose);
        Optional<FieldZone> lastZone = lastRobotPose.map((val) -> FieldZone.fromRobotPose(val));
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        boolean isInFriendlyZone = currentZone == FieldZone.BLUE && alliance == Alliance.Blue
                || currentZone == FieldZone.RED && alliance == Alliance.Red;
        boolean isInNeutralZone = currentZone == FieldZone.NEUTRAL;
        boolean isInEnemyZone = !isInFriendlyZone && !isInNeutralZone;

        // check if we have reached the final 3 seconds of the phase
        boolean hasReachedPrePhaseChange = false;
        if (timeLeftInPhase.isPresent()) {
            double prevSecsLeft =
                    lastTimeLeftInPhase.map((val) -> val.abs(Seconds)).orElse(140.0);
            if (timeLeftInPhase.get().abs(Seconds) <= 3
                    && prevSecsLeft > 3
                    && teleopPhase.get() != TeleopPhase.ENDGAME) {
                hasReachedPrePhaseChange = true;
            }
        }

        // TODO: change friendly zone shooter behavior based on whether our hub is active

        // check if our hub is active
        boolean isHubActive;
        if (autoWinner.isPresent() && teleopPhase.isPresent()) {
            isHubActive = isFriendlyHubActive(alliance, autoWinner.get(), teleopPhase.get());
        } else {
            isHubActive = true;
        }

        // check if hub status has changed
        boolean hasHubStateChanged = false;
        if (lastHubActive.isEmpty() || isHubActive != lastHubActive.get()) {
            hasHubStateChanged = true;
        }

        // check if the field zone has changed
        boolean hasZoneChanged = false;
        if (lastZone.isEmpty()
                || lastZone.get() != currentZone
                || lastTeleopPhase.isEmpty()
                || teleopPhase.orElse(TeleopPhase.TRANSITION_SHIFT) != lastTeleopPhase.get()) {
            hasZoneChanged = true;
        }

        // check if the teleop phase has changed
        boolean hasPhaseChanged = false;
        if (teleopPhase.isPresent() && teleopPhase.get() != lastTeleopPhase.orElse(TeleopPhase.ENDGAME)) {
            hasPhaseChanged = true;
        }

        if (hasZoneChanged || hasHubStateChanged && isInFriendlyZone) {
            enterFriendlyZone(isHubActive);
        } else if (hasZoneChanged && isInNeutralZone) {
            enterNeutralZone();
        } else if (hasZoneChanged && isInEnemyZone) {
            enterOpposingZone();
        }
        if (hasReachedPrePhaseChange) {
            CommandScheduler.getInstance()
                    .schedule(new DualBlinkLED(
                            RobotContainer.getInstance().ledSub, Color.kBlue, Color.kRed, timeLeftInPhase.get()));

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
        if (hasPhaseChanged && teleopPhase.get() == TeleopPhase.ENDGAME) {
            CommandScheduler.getInstance().schedule(new RainbowLED(RobotContainer.getInstance().ledSub, Seconds.of(5)));

            final double rumbleStrength = 0.75;
            ControllerUtil.scheduleControllerRumble(0, rumbleStrength, rumbleStrength, 1);
            ControllerUtil.scheduleControllerRumble(1, rumbleStrength, rumbleStrength, 1);
        }

        // log debug data
        Logger.logString(
                "", "teleopPhase", teleopPhase.map((val) -> val.toString()).orElse("None"));
        Logger.logDouble(
                "",
                "secsLeftInTeleopPhase",
                timeLeftInPhase.map((val) -> val.abs(Seconds)).orElse(Double.NaN));
        Logger.logBool("", "isFriendlyHubActive", isHubActive);

        lastRobotPose = Optional.of(robotPose);
        lastTimeLeftInPhase = timeLeftInPhase;
        lastTeleopPhase = teleopPhase;
        lastHubActive = Optional.of(isHubActive);
    }

    private void enterFriendlyZone(boolean isHubActive) {
        changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub,
                isHubActive
                        ? RobotContainer.getInstance().activeFriendlyZoneLEDCommand
                        : RobotContainer.getInstance().inactiveFriendlyZoneLEDCommand);

        // CommandScheduler.getInstance().schedule(RobotContainer.getInstance().spinUpIndexerCommand);
        // TODO: aim turret at hub
    }

    private void enterNeutralZone() {
        changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().neutralZoneLEDCommand);

        // CommandScheduler.getInstance().schedule(RobotContainer.getInstance().spinDownIndexerCommand);
        // TODO: aim turret at nearest gap to friendly alliance zone
    }

    private void enterOpposingZone() {
        changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().opposingZoneLEDCommand);

        // CommandScheduler.getInstance().schedule(RobotContainer.getInstance().spinDownIndexerCommand);
        // TODO: recenter turret
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

    private void changeSubsystemDefaultCommand(Subsystem sub, Command newDefault) {
        if (sub == null) return;

        Command currentDefault = sub.getDefaultCommand();
        Command currentCommand = sub.getCurrentCommand();

        if (currentDefault != null
                && currentCommand != null
                && currentDefault.getClass().equals(currentDefault.getClass())) {
            currentCommand.cancel();
        }
        sub.setDefaultCommand(newDefault);
    }

    @Override
    public void teleopExit() {}
}
