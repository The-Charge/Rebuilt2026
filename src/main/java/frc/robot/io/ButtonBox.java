package frc.robot.io;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import java.util.Map;

public class ButtonBox extends GenericHID {
    public enum Button {
        RESET_TURRET(1),
        UNJAM(2),
        UNUSED_3(3),
        DISABLE_ODO(4),
        TURRET_LEFT(5),
        TURRET_RIGHT(6),
        TEST_SHOOT(7),
        STOP_SHOOT(8),
        SPOOL_DOWN(9),
        SPOOL_UP(10);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        SLIDER(0);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    private static final InterpolatingDoubleTreeMap sliderInterpolater;

    static {
        sliderInterpolater = InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(-0.95, -1.0),
                Map.entry(-0.78, -0.5),
                Map.entry(-0.55, 0.0),
                Map.entry(0.7, 0.5),
                Map.entry(1.0, 1.0));
    }

    public ButtonBox(final int port) {
        super(port);
    }

    public double getSliderAxis() {
        return sliderInterpolater.get(getRawAxis(Axis.SLIDER.value));
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

    public boolean getUnjamButton() {
        return getRawButton(Button.UNJAM.value);
    }

    public boolean getUnjamButtonPressed() {
        return getRawButtonPressed(Button.UNJAM.value);
    }

    public boolean getUnjamButtonReleased() {
        return getRawButtonReleased(Button.UNJAM.value);
    }

    public BooleanEvent unjam(EventLoop loop) {
        return button(Button.UNJAM.value, loop);
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

    public boolean getDisableOdoButton() {
        return getRawButton(Button.DISABLE_ODO.value);
    }

    public boolean getDisabledOdoButtonPressed() {
        return getRawButtonPressed(Button.DISABLE_ODO.value);
    }

    public boolean getDisabledOdoButtonReleased() {
        return getRawButtonReleased(Button.DISABLE_ODO.value);
    }

    public BooleanEvent disabledOdo(EventLoop loop) {
        return button(Button.DISABLE_ODO.value, loop);
    }

    public boolean getTurretLeftButton() {
        return getRawButton(Button.TURRET_LEFT.value);
    }

    public boolean getTurretLeftButtonPressed() {
        return getRawButtonPressed(Button.TURRET_LEFT.value);
    }

    public boolean getTurretLeftButtonReleased() {
        return getRawButtonReleased(Button.TURRET_LEFT.value);
    }

    public BooleanEvent turretLeft(EventLoop loop) {
        return button(Button.TURRET_LEFT.value, loop);
    }

    public boolean getTurretRightButton() {
        return getRawButton(Button.TURRET_RIGHT.value);
    }

    public boolean getTurretRightButtonPressed() {
        return getRawButtonPressed(Button.TURRET_RIGHT.value);
    }

    public boolean getTurretRightButtonReleased() {
        return getRawButtonReleased(Button.TURRET_RIGHT.value);
    }

    public BooleanEvent turretRight(EventLoop loop) {
        return button(Button.TURRET_RIGHT.value, loop);
    }

    public boolean getTestShootButton() {
        return getRawButton(Button.TEST_SHOOT.value);
    }

    public boolean getTestShootButtonPressed() {
        return getRawButtonPressed(Button.TEST_SHOOT.value);
    }

    public boolean getTestButtonShootReleased() {
        return getRawButtonReleased(Button.TEST_SHOOT.value);
    }

    public BooleanEvent testShoot(EventLoop loop) {
        return button(Button.TEST_SHOOT.value, loop);
    }

    public boolean getStopShootButton() {
        return getRawButton(Button.STOP_SHOOT.value);
    }

    public boolean getStopShootButtonPressed() {
        return getRawButtonPressed(Button.STOP_SHOOT.value);
    }

    public boolean getStopShootButtonReleased() {
        return getRawButtonReleased(Button.STOP_SHOOT.value);
    }

    public BooleanEvent stopShoot(EventLoop loop) {
        return button(Button.STOP_SHOOT.value, loop);
    }

    public boolean getSpoolDownButton() {
        return getRawButton(Button.SPOOL_DOWN.value);
    }

    public boolean getSpoolDownButtonPressed() {
        return getRawButtonPressed(Button.SPOOL_DOWN.value);
    }

    public boolean getSpoolDownButtonReleased() {
        return getRawButtonReleased(Button.SPOOL_DOWN.value);
    }

    public BooleanEvent spoolDown(EventLoop loop) {
        return button(Button.SPOOL_DOWN.value, loop);
    }

    public boolean getSpoolUpButton() {
        return getRawButton(Button.SPOOL_UP.value);
    }

    public boolean getSpoolUpButtonPressed() {
        return getRawButtonPressed(Button.SPOOL_UP.value);
    }

    public boolean getSpoolUpButtonReleased() {
        return getRawButtonReleased(Button.SPOOL_UP.value);
    }

    public BooleanEvent spoolUp(EventLoop loop) {
        return button(Button.SPOOL_UP.value, loop);
    }
}
