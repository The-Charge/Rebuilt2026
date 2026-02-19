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

    public Trigger unjam() {
        return unjam(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger unjam(EventLoop loop) {
        return button(ButtonBox.Button.UNJAM.value, loop);
    }

    public Trigger unused3() {
        return unused3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger unused3(EventLoop loop) {
        return button(ButtonBox.Button.UNUSED_3.value, loop);
    }

    public Trigger disableOdo() {
        return disableOdo(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger disableOdo(EventLoop loop) {
        return button(ButtonBox.Button.DISABLE_ODO.value, loop);
    }

    public Trigger unused5() {
        return unused5(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger unused5(EventLoop loop) {
        return button(ButtonBox.Button.UNUSED_5.value, loop);
    }

    public Trigger unused6() {
        return unused6(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger unused6(EventLoop loop) {
        return button(ButtonBox.Button.UNUSED_6.value, loop);
    }

    public Trigger unused7() {
        return unused7(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger unused7(EventLoop loop) {
        return button(ButtonBox.Button.UNUSED_7.value, loop);
    }

    public Trigger ununsed8() {
        return ununsed8(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger ununsed8(EventLoop loop) {
        return button(ButtonBox.Button.UNUSED_8.value, loop);
    }

    public Trigger unused9() {
        return unused9(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger unused9(EventLoop loop) {
        return button(ButtonBox.Button.UNUSED_9.value, loop);
    }

    public Trigger unused10() {
        return unused10(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger unused10(EventLoop loop) {
        return button(ButtonBox.Button.UNUSED_10.value, loop);
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
