package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.ControllerUtil;
import frc.robot.utils.Logger;
import frc.robot.utils.MiscUtils;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class TeleopCommand extends Command {
    private SwerveRequest.FieldCentric swerveFieldCentricDrive;
    private SwerveRequest.RobotCentric swerveRobotCentricDrive;
    private SwerveRequest.FieldCentricFacingAngle swerveFieldCentricFacingAngleDrive;
    private SwerveRequest.FieldCentricFacingAngle swervePOVDrive;
    public SwerveRequest.SwerveDriveBrake swerveBrake;

    public Command swerveFieldCentricDriveCommand;
    public Command swerveRobotCentricDriveCommand;
    public Command swerveFieldCentricFacingAngleDriveCommand;
    public Command swervePOVDriveCommand;
    private Optional<Rotation2d> lastDefinedSnakeRotation = Optional.empty();
    private Optional<Rotation2d> lastDefinedPOVRotation = Optional.empty();

    private DoubleSupplier speedShifter, intakeMultiplier, cubicLeftY, cubicLeftX, linearRightX;

    private enum SwerveMode {
        BRAKE,
        ROBOTCENTRIC,
        FEILDCENTRIC,
        SNAKE,
        POV,
        OTHER,
    }

    private SwerveMode mode;
    private SwerveMode lastMode;

    public TeleopCommand() { // this badly needs organiztion
        mode = SwerveMode.FEILDCENTRIC;
        lastMode = SwerveMode.OTHER;

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

        setupCommands();

        RobotContainer.getInstance().swerve.setDefaultCommand(swerveFieldCentricDriveCommand);
    }

    private void setupCommands() {
        swerveFieldCentricDrive = new SwerveRequest.FieldCentric()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        swerveRobotCentricDrive = new SwerveRequest.RobotCentric()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        swerveFieldCentricFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
                .withHeadingPID(10, 0, 0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        swervePOVDrive = new SwerveRequest.FieldCentricFacingAngle()
                .withHeadingPID(10, 0, 0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        swerveBrake = new SwerveRequest.SwerveDriveBrake();

        swerveFieldCentricDriveCommand = RobotContainer.getInstance().swerve.applyRequest(() -> swerveFieldCentricDrive
                .withVelocityX(SwerveConstants.maxTranslationVel.times(
                        -cubicLeftY.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()))
                .withVelocityY(SwerveConstants.maxTranslationVel.times(
                        -cubicLeftX.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()))
                .withRotationalRate(
                        SwerveConstants.maxAngularVel.times(-linearRightX.getAsDouble() * speedShifter.getAsDouble())));
        swerveRobotCentricDriveCommand = RobotContainer.getInstance().swerve.applyRequest(() -> swerveRobotCentricDrive
                .withVelocityX(SwerveConstants.maxTranslationVel.times(
                        -cubicLeftY.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()))
                .withVelocityY(SwerveConstants.maxTranslationVel.times(
                        -cubicLeftX.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()))
                .withRotationalRate(
                        SwerveConstants.maxAngularVel.times(-linearRightX.getAsDouble() * speedShifter.getAsDouble())));
        swervePOVDriveCommand = RobotContainer.getInstance().swerve.applyRequest(() -> {
            int pov = RobotContainer.getInstance().hidDriver1.getPOV();
            if (pov != -1) {
                lastDefinedPOVRotation = Optional.of(new Rotation2d(Degrees.of(-pov)));
            }
            FieldCentricFacingAngle req = swervePOVDrive
                    .withVelocityX(SwerveConstants.maxTranslationVel.times(
                            -cubicLeftY.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()))
                    .withVelocityY(SwerveConstants.maxTranslationVel.times(
                            -cubicLeftX.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()));
            if (lastDefinedPOVRotation.isPresent()) {
                req = req.withTargetDirection(lastDefinedPOVRotation.get());
            }
            return req;
        });
        swerveFieldCentricFacingAngleDriveCommand = RobotContainer.getInstance()
                .swerve
                .applyRequest(() -> {
                    ChassisSpeeds speed = ChassisSpeeds.fromRobotRelativeSpeeds(
                            RobotContainer.getInstance().swerve.getStateCopy().Speeds,
                            RobotContainer.getInstance().swerve.getState().RawHeading);
                    if (Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) > 0.1) {
                        lastDefinedSnakeRotation = Optional.of(
                                new Rotation2d(Math.atan2(speed.vyMetersPerSecond, speed.vxMetersPerSecond)));
                    }

                    FieldCentricFacingAngle req = swerveFieldCentricFacingAngleDrive
                            .withVelocityX(SwerveConstants.maxTranslationVel.times(-cubicLeftY.getAsDouble()
                                    * speedShifter.getAsDouble()
                                    * intakeMultiplier.getAsDouble()))
                            .withVelocityY(SwerveConstants.maxTranslationVel.times(-cubicLeftX.getAsDouble()
                                    * speedShifter.getAsDouble()
                                    * intakeMultiplier.getAsDouble()));
                    if (lastDefinedSnakeRotation.isPresent()) {
                        req = req.withTargetDirection(lastDefinedSnakeRotation.get());
                    }

                    return req;
                });
    }

    public void teleopPeriodic() {
        if (RobotContainer.getInstance().hidDriver1.getXButton()) {
            mode = SwerveMode.BRAKE;
        } else if (RobotContainer.getInstance().hidDriver1.getLeftTriggerAxis() >= .5) {
            mode = SwerveMode.ROBOTCENTRIC;
        } else if (linearRightX.getAsDouble() != 0) {
            mode = SwerveMode.FEILDCENTRIC;
        } else if (RobotContainer.getInstance().hidDriver2.getLeftTriggerAxis() >= 0.5) {
            mode = SwerveMode.SNAKE;
        } else if (RobotContainer.getInstance().hidDriver1.getPOV() != -1) {
            mode = SwerveMode.POV;
        }
        Logger.logEnum("TeleopCommand", "Swerve Mode", mode);
        if (!mode.equals(lastMode)) {
            lastMode = mode;
            switch (mode) {
                case BRAKE:
                    RobotContainer.getInstance().swerve.applyRequest(() -> swerveBrake);
                    break;
                case ROBOTCENTRIC:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swerveRobotCentricDriveCommand, false);
                    break;
                case FEILDCENTRIC:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swerveFieldCentricDriveCommand, false);
                    break;
                case SNAKE:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swerveFieldCentricFacingAngleDriveCommand, false);
                    break;
                case POV:
                    MiscUtils.changeSubsystemDefaultCommand(
                            RobotContainer.getInstance().swerve, swervePOVDriveCommand, false);
                    break;
                default:
                    break;
            }
        }
    }

    public void endTeleop() {}
}
