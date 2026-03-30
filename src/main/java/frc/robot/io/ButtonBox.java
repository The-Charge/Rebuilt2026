package frc.robot.io;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import java.util.Map;

public class ButtonBox extends GenericHID {
    public enum Button {
        RESET_TURRET(1),
        ZERO_CLIMB(2),
        DEPLOY_INTAKE(3),
        ALT(4),
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

    private boolean alt() {
        return getRawButton(Button.ALT.value);
    }

    private boolean notAlt() {
        return !alt();
    }

    private boolean altPressed() {
        return getRawButtonPressed(Button.ALT.value);
    }

    private boolean altReleased() {
        return getRawButtonReleased(Button.ALT.value);
    }

    public double getSliderAxis() {
        return sliderInterpolater.get(getRawAxis(Axis.SLIDER.value));
    }

    public boolean getResetTurretButton() {
        return getRawButton(Button.RESET_TURRET.value) && notAlt();
    }

    public boolean getResetTurretButtonPressed() {
        return getRawButtonPressed(Button.RESET_TURRET.value) && notAlt();
    }

    public boolean getResetTurretButtonReleased() {
        return getRawButtonReleased(Button.RESET_TURRET.value)
                || (altPressed() && getRawButton(Button.RESET_TURRET.value));
    }

    public BooleanEvent resetTurret(EventLoop loop) {
        return button(Button.RESET_TURRET.value, loop).and(this::notAlt);
    }

    public boolean getSeedButton() {
        return getRawButton(Button.RESET_TURRET.value) && alt();
    }

    public boolean getSeedButtonPressed() {
        return getRawButtonPressed(Button.RESET_TURRET.value) && alt();
    }

    public boolean getSeedButtonReleased() {
        return getRawButtonReleased(Button.RESET_TURRET.value)
                || (altReleased() && getRawButton(Button.RESET_TURRET.value));
    }

    public BooleanEvent seed(EventLoop loop) {
        return button(Button.RESET_TURRET.value, loop).and(this::alt);
    }

    public boolean getZeroClimbButton() {
        return getRawButton(Button.ZERO_CLIMB.value) && notAlt();
    }

    public boolean getZeroClimbButtonPressed() {
        return getRawButtonPressed(Button.ZERO_CLIMB.value) && notAlt();
    }

    public boolean getZeroClimbButtonReleased() {
        return getRawButtonReleased(Button.ZERO_CLIMB.value) || (altPressed() && getRawButton(Button.ZERO_CLIMB.value));
    }

    public BooleanEvent zeroClimb(EventLoop loop) {
        return button(Button.ZERO_CLIMB.value, loop).and(this::notAlt);
    }

    public boolean getDeployIntakeButton() {
        return getRawButton(Button.DEPLOY_INTAKE.value) && notAlt();
    }

    public boolean getDeployIntakeButtonPressed() {
        return getRawButtonPressed(Button.DEPLOY_INTAKE.value) && notAlt();
    }

    public boolean getDeployIntakeButtonReleased() {
        return getRawButtonReleased(Button.DEPLOY_INTAKE.value)
                || (altPressed() && getRawButton(Button.DEPLOY_INTAKE.value));
    }

    public BooleanEvent deployIntake(EventLoop loop) {
        return button(Button.DEPLOY_INTAKE.value, loop).and(this::notAlt);
    }

    public boolean getAltButton() {
        return getRawButton(Button.ALT.value);
    }

    public boolean getAltButtonPressed() {
        return getRawButtonPressed(Button.ALT.value);
    }

    public boolean getAltButtonReleased() {
        return getRawButtonReleased(Button.ALT.value);
    }

    public BooleanEvent alt(EventLoop loop) {
        return button(Button.ALT.value, loop);
    }

    public boolean getTurretLeftButton() {
        return getRawButton(Button.TURRET_LEFT.value) && notAlt();
    }

    public boolean getTurretLeftButtonPressed() {
        return getRawButtonPressed(Button.TURRET_LEFT.value) && notAlt();
    }

    public boolean getTurretLeftButtonReleased() {
        return getRawButtonReleased(Button.TURRET_LEFT.value)
                || (altPressed() && getRawButton(Button.TURRET_LEFT.value));
    }

    public BooleanEvent turretLeft(EventLoop loop) {
        return button(Button.TURRET_LEFT.value, loop).and(this::notAlt);
    }

    public boolean getTurretRightButton() {
        return getRawButton(Button.TURRET_RIGHT.value) && notAlt();
    }

    public boolean getTurretRightButtonPressed() {
        return getRawButtonPressed(Button.TURRET_RIGHT.value) && notAlt();
    }

    public boolean getTurretRightButtonReleased() {
        return getRawButtonReleased(Button.TURRET_RIGHT.value)
                || (altPressed() && getRawButton(Button.TURRET_RIGHT.value));
    }

    public BooleanEvent turretRight(EventLoop loop) {
        return button(Button.TURRET_RIGHT.value, loop).and(this::notAlt);
    }

    public boolean getTestShootButton() {
        return getRawButton(Button.TEST_SHOOT.value) && notAlt();
    }

    public boolean getTestShootButtonPressed() {
        return getRawButtonPressed(Button.TEST_SHOOT.value) && notAlt();
    }

    public boolean getTestShootButtonReleased() {
        return getRawButtonReleased(Button.TEST_SHOOT.value) || (altPressed() && getRawButton(Button.TEST_SHOOT.value));
    }

    public BooleanEvent testShoot(EventLoop loop) {
        return button(Button.TEST_SHOOT.value, loop).and(this::notAlt);
    }

    public boolean getStopShootButton() {
        return getRawButton(Button.STOP_SHOOT.value) && notAlt();
    }

    public boolean getStopShootButtonPressed() {
        return getRawButtonPressed(Button.STOP_SHOOT.value) && notAlt();
    }

    public boolean getStopShootButtonReleased() {
        return getRawButtonReleased(Button.STOP_SHOOT.value) || (altPressed() && getRawButton(Button.STOP_SHOOT.value));
    }

    public BooleanEvent stopShoot(EventLoop loop) {
        return button(Button.STOP_SHOOT.value, loop).and(this::notAlt);
    }

    public boolean getSpoolDownButton() {
        return getRawButton(Button.SPOOL_DOWN.value) && notAlt();
    }

    public boolean getSpoolDownButtonPressed() {
        return getRawButtonPressed(Button.SPOOL_DOWN.value) && notAlt();
    }

    public boolean getSpoolDownButtonReleased() {
        return getRawButtonReleased(Button.SPOOL_DOWN.value) || (altPressed() && getRawButton(Button.SPOOL_DOWN.value));
    }

    public BooleanEvent spoolDown(EventLoop loop) {
        return button(Button.SPOOL_DOWN.value, loop).and(this::notAlt);
    }

    public boolean getSpoolUpButton() {
        return getRawButton(Button.SPOOL_UP.value) && notAlt();
    }

    public boolean getSpoolUpButtonPressed() {
        return getRawButtonPressed(Button.SPOOL_UP.value) && notAlt();
    }

    public boolean getSpoolUpButtonReleased() {
        return getRawButtonReleased(Button.SPOOL_UP.value) || (altPressed() && getRawButton(Button.SPOOL_UP.value));
    }

    public BooleanEvent spoolUp(EventLoop loop) {
        return button(Button.SPOOL_UP.value, loop).and(this::notAlt);
    }
}
