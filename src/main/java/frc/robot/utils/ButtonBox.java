package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class ButtonBox extends GenericHID {
    public enum Button {
        RESET_TURRET(1),
        UNUSED_2(2),
        UNUSED_3(3),
        UNUSED_4(4),
        UNUSED_5(5),
        UNUSED_6(6),
        UNUSED_7(7),
        UNUSED_8(8),
        UNUSED_9(9),
        UNUSED_10(10);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        UNUSED_0(0);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public ButtonBox(final int port) {
        super(port);
    }

    public double getUnused0Axis() {
        return getRawAxis(Axis.UNUSED_0.value);
    }

    public boolean getResetTurretButton() {
        return getRawButton(Button.RESET_TURRET.value);
    }

    public boolean getResetTurretButtonPressed() {
        return getRawButtonPressed(Button.RESET_TURRET.value);
    }

    public boolean getResetTurretButtonReleased() {
        return getRawButtonReleased(Button.RESET_TURRET.value);
    }

    public BooleanEvent resetTurret(EventLoop loop) {
        return button(Button.RESET_TURRET.value, loop);
    }

    public boolean getUnused2Button() {
        return getRawButton(Button.UNUSED_2.value);
    }

    public boolean getUnused2ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_2.value);
    }

    public boolean getUnused2ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_2.value);
    }

    public BooleanEvent unused2(EventLoop loop) {
        return button(Button.UNUSED_2.value, loop);
    }

    public boolean getUnused3Button() {
        return getRawButton(Button.UNUSED_3.value);
    }

    public boolean getUnused3ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_3.value);
    }

    public boolean getUnusedButton3Released() {
        return getRawButtonReleased(Button.UNUSED_3.value);
    }

    public BooleanEvent unused3(EventLoop loop) {
        return button(Button.UNUSED_3.value, loop);
    }

    public boolean getUnused4Button() {
        return getRawButton(Button.UNUSED_4.value);
    }

    public boolean getUnused4ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_4.value);
    }

    public boolean getUnused4ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_4.value);
    }

    public BooleanEvent unused4(EventLoop loop) {
        return button(Button.UNUSED_4.value, loop);
    }

    public boolean getUnused5Button() {
        return getRawButton(Button.UNUSED_5.value);
    }

    public boolean getUnused5ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_5.value);
    }

    public boolean getUnused5ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_5.value);
    }

    public BooleanEvent unused5(EventLoop loop) {
        return button(Button.UNUSED_5.value, loop);
    }

    public boolean getUnused6Button() {
        return getRawButton(Button.UNUSED_6.value);
    }

    public boolean getUnused6ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_6.value);
    }

    public boolean getUnused6ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_6.value);
    }

    public BooleanEvent unused6(EventLoop loop) {
        return button(Button.UNUSED_6.value, loop);
    }

    public boolean getUnused7Button() {
        return getRawButton(Button.UNUSED_7.value);
    }

    public boolean getUnused7ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_7.value);
    }

    public boolean getUnused7ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_7.value);
    }

    public BooleanEvent unused7(EventLoop loop) {
        return button(Button.UNUSED_7.value, loop);
    }

    public boolean getUnused8Button() {
        return getRawButton(Button.UNUSED_8.value);
    }

    public boolean getUnused8ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_8.value);
    }

    public boolean getUnused8ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_8.value);
    }

    public BooleanEvent unused8(EventLoop loop) {
        return button(Button.UNUSED_8.value, loop);
    }

    public boolean getUnused9Button() {
        return getRawButton(Button.UNUSED_9.value);
    }

    public boolean getUnused9ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_9.value);
    }

    public boolean getUnused9ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_9.value);
    }

    public BooleanEvent unused9(EventLoop loop) {
        return button(Button.UNUSED_9.value, loop);
    }

    public boolean getUnused10Button() {
        return getRawButton(Button.UNUSED_10.value);
    }

    public boolean getUnused10ButtonPressed() {
        return getRawButtonPressed(Button.UNUSED_10.value);
    }

    public boolean getUnused10ButtonReleased() {
        return getRawButtonReleased(Button.UNUSED_10.value);
    }

    public BooleanEvent ununsed10(EventLoop loop) {
        return button(Button.UNUSED_10.value, loop);
    }
}
