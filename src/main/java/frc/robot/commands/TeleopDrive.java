package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.ControllerUtil;
import frc.robot.utils.Logger;
import frc.robot.utils.MiscUtils;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopDrive {

    private enum SwerveMode {
        BRAKE,
        ROBOT_CENTRIC_POV,
        FIELD_CENTRIC_POV,
        ROBOT_CENTRIC_OMEGA,
        FIELD_CENTRIC_OMEGA,
        SNAKE
    }

    private SwerveRequest.FieldCentric fieldCentricOmegaRequest;
    private SwerveRequest.RobotCentric robotCentricOmegaRequest;
    private SwerveRequest.FieldCentricFacingAngle fieldCentricPOVRequest;
    private SwerveRequest.RobotCentricFacingAngle robotCentricPOVRequest;
    private SwerveRequest.SwerveDriveBrake brakeRequest;

    private Command fieldCentricOmegaCommand;
    private Command robotCentricOmegaCommand;
    private Command fieldCentricPOVCommand;
    private Command robotCentricPOVCommand;
    private Command snakeCommand;
    private Command brakeCommand;

    private final DoubleSupplier speedShifter, slowDown, cubicLeftY, cubicLeftX;
    private final Supplier<Integer> omegaTurnDirection;
    private final Supplier<LinearVelocity> commonXVel, commonYVel;
    private final Supplier<AngularVelocity> commonOmegaVel;
    private final Supplier<Optional<Angle>> commonPOVAngle;

    private Optional<SwerveMode> lastMode;
    private Optional<Rotation2d> lastDefinedSnakeRotation;
    private Optional<Rotation2d> lastDefinedPOVRotation;

    public TeleopDrive() {
        speedShifter = () -> RobotContainer.getInstance().hidDriver1.getRightTriggerAxis() >= 0.5 ? 0.25 : 1;
        slowDown = () -> {
            if (RobotContainer.getInstance().hidDriver2.getRightTriggerAxis() >= 0.5) {
                return 0.25 * 0.5;
            } else if (RobotContainer.getInstance().hidDriver2.getRightTriggerAxis() >= 0.5) {
                return 0.5;
            }
            return 1;
        };

        cubicLeftY = () -> ControllerUtil.applyExponentialDeadband(
                RobotContainer.getInstance().hidDriver1.getLeftY(),
                SwerveConstants.driveJoystickDeadband,
                SwerveConstants.joystickExponent);
        cubicLeftX = () -> ControllerUtil.applyExponentialDeadband(
                RobotContainer.getInstance().hidDriver1.getLeftX(),
                SwerveConstants.driveJoystickDeadband,
                SwerveConstants.joystickExponent);
        omegaTurnDirection = () -> {
            int povVal = RobotContainer.getInstance().hidDriver1.getPOV();

            switch (povVal) {
                case 90:
                    return -1;
                case 270:
                    return 1;
                default:
                    return 0;
            }
        };

        commonXVel = () -> SwerveConstants.maxTranslationVel.times(
                -cubicLeftY.getAsDouble() * speedShifter.getAsDouble() * slowDown.getAsDouble());
        commonYVel = () -> SwerveConstants.maxTranslationVel.times(
                -cubicLeftX.getAsDouble() * speedShifter.getAsDouble() * slowDown.getAsDouble());
        commonOmegaVel =
                () -> SwerveConstants.maxAngularVel.times(omegaTurnDirection.get() * speedShifter.getAsDouble());
        commonPOVAngle = () -> {
            double x = RobotContainer.getInstance().hidDriver1.getRightX();
            double y = RobotContainer.getInstance().hidDriver1.getRightY();

            if (Math.hypot(x, y) < SwerveConstants.turnJoystickDeadband) return Optional.empty();

            return Optional.of(Radians.of(Math.atan2(-x, -y)));
        };

        lastMode = Optional.empty();
        lastDefinedSnakeRotation = Optional.empty();
        lastDefinedPOVRotation = Optional.empty();

        setupCommands();

        RobotContainer.getInstance().swerve.setDefaultCommand(fieldCentricPOVCommand);
    }

    public String getName() {
        return "TelopDrive";
    }

    private void setupCommands() {
        fieldCentricOmegaRequest = new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.deadbandTranslationVel)
                .withRotationalDeadband(SwerveConstants.deadbandAngularVel)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        robotCentricOmegaRequest = new SwerveRequest.RobotCentric()
                .withDeadband(SwerveConstants.deadbandTranslationVel)
                .withRotationalDeadband(SwerveConstants.deadbandAngularVel)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        fieldCentricPOVRequest = new SwerveRequest.FieldCentricFacingAngle()
                .withHeadingPID(
                        SwerveConstants.headingPID.kP, SwerveConstants.headingPID.kI, SwerveConstants.headingPID.kD)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        robotCentricPOVRequest = new SwerveRequest.RobotCentricFacingAngle()
                .withHeadingPID(
                        SwerveConstants.headingPID.kP, SwerveConstants.headingPID.kI, SwerveConstants.headingPID.kD)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        brakeRequest = new SwerveRequest.SwerveDriveBrake();

        fieldCentricOmegaCommand = RobotContainer.getInstance().swerve.applyRequest(() -> fieldCentricOmegaRequest
                .withVelocityX(commonXVel.get())
                .withVelocityY(commonYVel.get())
                .withRotationalRate(commonOmegaVel.get()));
        robotCentricOmegaCommand = RobotContainer.getInstance().swerve.applyRequest(() -> robotCentricOmegaRequest
                .withVelocityX(commonXVel.get())
                .withVelocityY(commonYVel.get())
                .withRotationalRate(commonOmegaVel.get()));
        fieldCentricPOVCommand = RobotContainer.getInstance().swerve.applyRequest(() -> {
            Optional<Angle> povAngle = commonPOVAngle.get();
            if (povAngle.isPresent()) {
                lastDefinedPOVRotation = Optional.of(new Rotation2d(povAngle.get()));
            }

            FieldCentricFacingAngle req =
                    fieldCentricPOVRequest.withVelocityX(commonXVel.get()).withVelocityY(commonYVel.get());
            if (lastDefinedPOVRotation.isPresent()) {
                req.withTargetDirection(lastDefinedPOVRotation.get());
            } else {
                req.withTargetDirection(
                        RobotContainer.getInstance().swerve.getState().Pose.getRotation());
            }

            return req;
        });
        robotCentricPOVCommand = RobotContainer.getInstance().swerve.applyRequest(() -> {
            Optional<Angle> povAngle = commonPOVAngle.get();
            if (povAngle.isPresent()) {
                lastDefinedPOVRotation = Optional.of(new Rotation2d(povAngle.get()));
            }

            RobotCentricFacingAngle req =
                    robotCentricPOVRequest.withVelocityX(commonXVel.get()).withVelocityY(commonYVel.get());
            if (lastDefinedPOVRotation.isPresent()) {
                req.withTargetDirection(lastDefinedPOVRotation.get());
            } else {
                req.withTargetDirection(
                        RobotContainer.getInstance().swerve.getState().Pose.getRotation());
            }

            return req;
        });
        snakeCommand = RobotContainer.getInstance().swerve.applyRequest(() -> {
            if (Math.hypot(cubicLeftX.getAsDouble(), cubicLeftY.getAsDouble()) > 0.1) {
                lastDefinedSnakeRotation =
                        Optional.of(new Rotation2d(Math.atan2(-cubicLeftX.getAsDouble(), -cubicLeftY.getAsDouble())));
            }

            FieldCentricFacingAngle req =
                    fieldCentricPOVRequest.withVelocityX(commonXVel.get()).withVelocityY(commonYVel.get());
            if (lastDefinedSnakeRotation.isPresent()) {
                req.withTargetDirection(lastDefinedSnakeRotation.get());
            } else {
                req.withTargetDirection(
                        RobotContainer.getInstance().swerve.getState().Pose.getRotation());
            }

            return req;
        });
        brakeCommand = RobotContainer.getInstance().swerve.applyRequest(() -> brakeRequest);
    }

    public void teleopPeriodic() {
        Logger.logDouble(getName(), "speedShifter", speedShifter.getAsDouble());
        Logger.logDouble(getName(), "intakeMultiplier", slowDown.getAsDouble());
        Logger.logDouble(getName(), "cubicLeftY", cubicLeftY.getAsDouble());
        Logger.logDouble(getName(), "cubicLeftX", cubicLeftX.getAsDouble());
        Logger.logLong(getName(), "omegaTurnDirection", omegaTurnDirection.get());
        Logger.logDouble(getName(), "commonXVel", commonXVel.get().in(MetersPerSecond));
        Logger.logDouble(getName(), "commonYVel", commonYVel.get().in(MetersPerSecond));
        Logger.logDouble(getName(), "commonOmegaVel", commonOmegaVel.get().in(RadiansPerSecond));
        Logger.logDouble(
                getName(),
                "commonPOVAngle",
                commonPOVAngle.get().map((val) -> val.in(Degrees)).orElse(Double.NaN));

        SwerveMode mode;
        if (RobotContainer.getInstance().hidDriver1.getXButton()) {
            mode = SwerveMode.BRAKE;
        } else if (RobotContainer.getInstance().hidDriver1.getLeftTriggerAxis() >= 0.5) {
            if (commonPOVAngle.get().isPresent()) {
                mode = SwerveMode.ROBOT_CENTRIC_POV;
            } else {
                mode = SwerveMode.ROBOT_CENTRIC_OMEGA;
            }
        } else if (RobotContainer.getInstance().hidDriver1.getLeftBumperButton()) {
            mode = SwerveMode.SNAKE;
        } else {
            if (commonPOVAngle.get().isPresent()) {
                mode = SwerveMode.FIELD_CENTRIC_POV;
            } else {
                mode = SwerveMode.FIELD_CENTRIC_OMEGA;
            }
        }

        Logger.logEnum(getName(), "mode", mode);

        if (lastMode.isEmpty() || !mode.equals(lastMode.get())) {
            lastMode = Optional.of(mode);

            switch (mode) {
                case BRAKE:
                    MiscUtils.changeSubsystemDefaultCommand(RobotContainer.getInstance().swerve, brakeCommand, true);
                    break;
                case ROBOT_CENTRIC_OMEGA:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, robotCentricOmegaCommand, true);
                    break;
                case ROBOT_CENTRIC_POV:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, robotCentricPOVCommand, true);
                    break;
                case FIELD_CENTRIC_OMEGA:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, fieldCentricOmegaCommand, true);
                    break;
                case FIELD_CENTRIC_POV:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, fieldCentricPOVCommand, true);
                    break;
                case SNAKE:
                    MiscUtils.changeSubsystemDefaultCommand(RobotContainer.getInstance().swerve, snakeCommand, true);
                    break;
                default:
                    break;
            }
        }
    }

    public void endTeleop() {
        MiscUtils.removeSubsystemDefaultCommand(RobotContainer.getInstance().swerve, true);
    }
}
