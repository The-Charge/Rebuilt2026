package frc.robot.io;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonBox extends CommandGenericHID {

    private final ButtonBox m_hid;

    public CommandButtonBox(int port) {
        super(port);
        m_hid = new ButtonBox(port);
    }

    @Override
    public ButtonBox getHID() {
        return m_hid;
    }

    public Trigger resetTurret() {
        return resetTurret(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger resetTurret(EventLoop loop) {
        return button(ButtonBox.Button.RESET_TURRET.value, loop);
    }

    public Trigger zeroClimb() {
        return zeroClimb(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger zeroClimb(EventLoop loop) {
        return button(ButtonBox.Button.ZERO_CLIMB.value, loop);
    }

    public Trigger deployIntake() {
        return deployIntake(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger deployIntake(EventLoop loop) {
        return button(ButtonBox.Button.DEPLOY_INTAKE.value, loop);
    }

    public Trigger seed() {
        return seed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger seed(EventLoop loop) {
        return new Trigger(m_hid.seed(loop));
    }

    public Trigger turretLeft() {
        return turretLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger turretLeft(EventLoop loop) {
        return button(ButtonBox.Button.TURRET_LEFT.value, loop);
    }

    public Trigger turretRight() {
        return turretRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger turretRight(EventLoop loop) {
        return button(ButtonBox.Button.TURRET_RIGHT.value, loop);
    }

    public Trigger testShoot() {
        return testShoot(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger testShoot(EventLoop loop) {
        return button(ButtonBox.Button.TEST_SHOOT.value, loop);
    }

    public Trigger stopShoot() {
        return stopShoot(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger stopShoot(EventLoop loop) {
        return button(ButtonBox.Button.STOP_SHOOT.value, loop);
    }

    public Trigger spoolDown() {
        return spoolDown(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger spoolDown(EventLoop loop) {
        return button(ButtonBox.Button.SPOOL_DOWN.value, loop);
    }

    public Trigger spoolUp() {
        return spoolUp(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger spoolUp(EventLoop loop) {
        return button(ButtonBox.Button.SPOOL_UP.value, loop);
    }

    public Trigger slider(double threshold, EventLoop loop) {
        return axisGreaterThan(ButtonBox.Axis.SLIDER.value, threshold, loop);
    }

    public Trigger slider(double threshold) {
        return slider(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger slider() {
        return slider(0.5);
    }

    public double getSlider() {
        return m_hid.getSliderAxis();
    }
}
