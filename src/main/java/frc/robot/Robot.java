// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    public Robot() {
        Logger.init(); // DO NOT DELETE ; start logger
        RobotContainer.getInstance(); // DO NOT DELETE ; create singleton instance
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.logPDP(RobotContainer.getInstance().pdp);

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
    }

    @Override
    public void disabledInit() {
        RobotContainer.getInstance().indexer.stopAll();
        RobotContainer.getInstance().intake.stopRoller();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();

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
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
