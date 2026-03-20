package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.ControllerUtil;
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

    public TeleopCommand() {
        DoubleSupplier speedShifter =
                () -> RobotContainer.getInstance().hidDriver1.getRightTriggerAxis() >= 0.5 ? 0.25 : 1;
        DoubleSupplier intakeMultiplier =
                () -> RobotContainer.getInstance().hidDriver2.getLeftTriggerAxis() >= 0.5 ? 0.5 : 1;

        DoubleSupplier cubicLeftY = () -> ControllerUtil.applyExponentialDeadband(
                RobotContainer.getInstance().hidDriver1.getLeftY(),
                SwerveConstants.joystickDeadband,
                SwerveConstants.joystickExponent);
        DoubleSupplier cubicLeftX = () -> ControllerUtil.applyExponentialDeadband(
                RobotContainer.getInstance().hidDriver1.getLeftX(),
                SwerveConstants.joystickDeadband,
                SwerveConstants.joystickExponent);
        DoubleSupplier linearRightX = () -> ControllerUtil.applyLinearDeadband(
                RobotContainer.getInstance().hidDriver1.getRightX(), SwerveConstants.joystickDeadband);

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
            return swervePOVDrive
                    .withTargetDirection(new Rotation2d(
                            Degrees.of(-RobotContainer.getInstance().hidDriver1.getPOV())))
                    .withVelocityX(SwerveConstants.maxTranslationVel.times(
                            -cubicLeftY.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()))
                    .withVelocityY(SwerveConstants.maxTranslationVel.times(
                            -cubicLeftX.getAsDouble() * speedShifter.getAsDouble() * intakeMultiplier.getAsDouble()));
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

                    var req = swerveFieldCentricFacingAngleDrive
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

        RobotContainer.getInstance().swerve.setDefaultCommand(swerveFieldCentricDriveCommand);
    }

    public void teleopPeriodic() {
        if (RobotContainer.getInstance().hidDriver1.getLeftBumperButton()) {

        }
    }

    public void endTeleop() {}
}
