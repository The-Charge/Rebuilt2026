package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
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
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        SNAKE,
        POV
    }

    private SwerveRequest.FieldCentric swerveFieldCentricDrive;
    private SwerveRequest.RobotCentric swerveRobotCentricDrive;
    private SwerveRequest.FieldCentricFacingAngle swerveFieldCentricFacingAngleDrive;
    private SwerveRequest.FieldCentricFacingAngle swervePOVDrive;
    private SwerveRequest.SwerveDriveBrake swerveBrake;

    private Command swerveFieldCentricDriveCommand;
    private Command swerveRobotCentricDriveCommand;
    private Command swerveFieldCentricFacingAngleDriveCommand;
    private Command swervePOVDriveCommand;

    private final DoubleSupplier speedShifter, intakeMultiplier, cubicLeftY, cubicLeftX, linearRightX;
    private final Supplier<LinearVelocity> commonXVel, commonYVel;
    private final Supplier<AngularVelocity> commonOmegaVel;

    private Optional<SwerveMode> lastMode;
    private Optional<Rotation2d> lastDefinedSnakeRotation;
    private Optional<Rotation2d> lastDefinedPOVRotation;

    public TeleopDrive() {
        speedShifter = () -> RobotContainer.getInstance().hidDriver1.getRightTriggerAxis() >= 0.5 ? 0.25 : 1;
        intakeMultiplier = () -> RobotContainer.getInstance().hidDriver2.getLeftTriggerAxis() >= 0.5 ? 0.5 : 1;

        cubicLeftY = () -> ControllerUtil.applyExponentialDeadband(
                RobotContainer.getInstance().hidDriver1.getLeftY(),
                SwerveConstants.joystickDeadband,
                SwerveConstants.joystickExponent);
        cubicLeftX = () -> ControllerUtil.applyExponentialDeadband(
                RobotContainer.getInstance().hidDriver1.getLeftX(),
                SwerveConstants.joystickDeadband,
                SwerveConstants.joystickExponent);
        linearRightX = () -> ControllerUtil.applyLinearDeadband(
                RobotContainer.getInstance().hidDriver1.getRightX(), SwerveConstants.joystickDeadband);

        commonXVel = () -> SwerveConstants.maxTranslationVel.times(
                -cubicLeftY.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble());
        commonYVel = () -> SwerveConstants.maxTranslationVel.times(
                -cubicLeftX.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble());
        commonOmegaVel =
                () -> SwerveConstants.maxAngularVel.times(-linearRightX.getAsDouble() * speedShifter.getAsDouble());

        lastMode = Optional.empty();
        lastDefinedSnakeRotation = Optional.empty();
        lastDefinedPOVRotation = Optional.empty();

        setupCommands();

        RobotContainer.getInstance().swerve.setDefaultCommand(swerveFieldCentricDriveCommand);
    }

    private void setupCommands() {
        swerveFieldCentricDrive = new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.deadbandTranslationVel)
                .withRotationalDeadband(SwerveConstants.deadbandAngularVel)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        swerveRobotCentricDrive = new SwerveRequest.RobotCentric()
                .withDeadband(SwerveConstants.deadbandTranslationVel)
                .withRotationalDeadband(SwerveConstants.deadbandAngularVel)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        swerveFieldCentricFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
                .withHeadingPID(
                        SwerveConstants.headingPID.kP, SwerveConstants.headingPID.kI, SwerveConstants.headingPID.kD)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        swervePOVDrive = new SwerveRequest.FieldCentricFacingAngle()
                .withHeadingPID(
                        SwerveConstants.headingPID.kP, SwerveConstants.headingPID.kI, SwerveConstants.headingPID.kD)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        swerveBrake = new SwerveRequest.SwerveDriveBrake();

        swerveFieldCentricDriveCommand = RobotContainer.getInstance().swerve.applyRequest(() -> swerveFieldCentricDrive
                .withVelocityX(commonXVel.get())
                .withVelocityY(commonYVel.get())
                .withRotationalRate(commonOmegaVel.get()));
        swerveRobotCentricDriveCommand = RobotContainer.getInstance().swerve.applyRequest(() -> swerveRobotCentricDrive
                .withVelocityX(commonXVel.get())
                .withVelocityY(commonYVel.get())
                .withRotationalRate(commonOmegaVel.get()));
        swervePOVDriveCommand = RobotContainer.getInstance().swerve.applyRequest(() -> {
            int pov = RobotContainer.getInstance().hidDriver1.getPOV();
            if (pov != -1) {
                lastDefinedPOVRotation = Optional.of(new Rotation2d(Degrees.of(-pov)));
            }

            FieldCentricFacingAngle req =
                    swervePOVDrive.withVelocityX(commonXVel.get()).withVelocityY(commonYVel.get());
            if (lastDefinedPOVRotation.isPresent()) {
                req.withTargetDirection(lastDefinedPOVRotation.get());
            }

            return req;
        });
        swerveFieldCentricFacingAngleDriveCommand = RobotContainer.getInstance()
                .swerve
                .applyRequest(() -> {
                    SwerveDriveState state = RobotContainer.getInstance().swerve.getState();
                    // ChassisSpeeds speed = ChassisSpeeds.fromRobotRelativeSpeeds(
                    //         state.Speeds, state.Pose.getRotation()); // TODO: make sure rotation works on red
                    // alliance
                    if (Math.hypot(cubicLeftX.getAsDouble(), cubicLeftY.getAsDouble()) > 0.1) {
                        lastDefinedSnakeRotation = Optional.of(
                                new Rotation2d(Math.atan2(-cubicLeftX.getAsDouble(), -cubicLeftY.getAsDouble())));
                    }

                    FieldCentricFacingAngle req = swerveFieldCentricFacingAngleDrive
                            .withVelocityX(commonXVel.get())
                            .withVelocityY(commonYVel.get());
                    if (lastDefinedSnakeRotation.isPresent()) {
                        req.withTargetDirection(lastDefinedSnakeRotation.get());
                    }

                    return req;
                });
    }

    public void teleopPeriodic() {
        SwerveMode mode;
        if (RobotContainer.getInstance().hidDriver1.getXButton()) {
            mode = SwerveMode.BRAKE;
        } else if (RobotContainer.getInstance().hidDriver1.getLeftTriggerAxis() >= 0.5) {
            mode = SwerveMode.ROBOT_CENTRIC;
        } else if (RobotContainer.getInstance().hidDriver1.getLeftBumperButton()) {
            mode = SwerveMode.SNAKE;
        } else if (RobotContainer.getInstance().hidDriver1.getPOV() != -1
                || (lastMode.isPresent() && lastMode.get().equals(SwerveMode.POV) && linearRightX.getAsDouble() == 0)) {
            mode = SwerveMode.POV;
        } else {
            mode = SwerveMode.FIELD_CENTRIC;
        }

        Logger.logEnum("TeleopDrive", "mode", mode);

        if (lastMode.isEmpty() || !mode.equals(lastMode.get())) {
            lastMode = Optional.of(mode);

            switch (mode) {
                case BRAKE:
                    RobotContainer.getInstance().swerve.applyRequest(() -> swerveBrake);
                    break;
                case ROBOT_CENTRIC:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swerveRobotCentricDriveCommand, true);
                    break;
                case FIELD_CENTRIC:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swerveFieldCentricDriveCommand, true);
                    break;
                case SNAKE:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swerveFieldCentricFacingAngleDriveCommand, true);
                    break;
                case POV:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swervePOVDriveCommand, true);
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
