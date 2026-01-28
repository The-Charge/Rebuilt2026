package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TeleopDrive extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier heading;
    private final IntSupplier pov;
    private final DoubleSupplier shiftScalar;
    private final BooleanSupplier reefLock;
    private final BooleanSupplier centricToggle;

    private boolean isFieldCentric = true;
    private boolean centricToggleLast = false;

    private enum Mode {
        JOYSTICK,
        POV,
        REEF
    };

    private Mode mode = Mode.JOYSTICK;

    public TeleopDrive(
            SwerveSubsystem swerve,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier heading,
            IntSupplier pov,
            BooleanSupplier reefLock,
            BooleanSupplier centricToggle,
            DoubleSupplier shift) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;
        this.pov = pov;
        this.reefLock = reefLock;
        this.centricToggle = centricToggle;
        this.shiftScalar = shift;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Swerve IsFieldCentric", isFieldCentric);
    }

    @Override
    public void execute() {
        int povVal = pov.getAsInt();
        boolean centricToggleVal = centricToggle.getAsBoolean();
        boolean reefLockVal = reefLock.getAsBoolean();
        double headingVal = heading.getAsDouble();
        double shiftScalarVal = shiftScalar.getAsDouble();
        double vXVal = vX.getAsDouble();
        double vYVal = vY.getAsDouble();

        // Handle field-centric toggle
        if (centricToggleVal && !centricToggleLast) {
            isFieldCentric = !isFieldCentric;
            SmartDashboard.putBoolean("Swerve IsFieldCentric", isFieldCentric);
        }
        centricToggleLast = centricToggleVal;

        // reefLock get priority, then POV, then normal
        if (reefLockVal) {
            mode = Mode.REEF;
        } else if (povVal != -1) {
            mode = Mode.POV;
        } else if (Math.abs(headingVal) >= SwerveConstants.RIGHT_X_DEADBAND) {
            mode = Mode.JOYSTICK;
        }
        SmartDashboard.putString("Teleop Mode", mode.name());
        // no else statement, just leave as last value

        // Calculate speed multiplier
        double shiftAmt = -0.9 * shiftScalarVal + 1;

        // Calculate translation
        double transMult = SwerveConstants.MAX_SPEED * shiftAmt * SwerveConstants.DRIVE_SPEED;
        transMult *= isFieldCentric ? (swerve.isRedAlliance() ? -1 : 1) : 1;
        Translation2d translation = new Translation2d(vXVal * transMult, vYVal * transMult);

        switch (mode) {
            case POV:
                // Drive with POV
                swerve.drive(translation, POVDrive(povVal, vXVal, vYVal), isFieldCentric);
                break;
            case REEF:
                // Drive with Reef Rotation
                swerve.drive(translation, ReefLock(vXVal, vYVal), isFieldCentric);
                break;
            default:
                // Drive normally
                double rotationSpeed = headingVal * swerve.getSwerveController().config.maxAngularVelocity;
                rotationSpeed *= shiftAmt * SwerveConstants.DRIVE_SPEED;

                swerve.drive(translation, rotationSpeed, isFieldCentric);
                break;
        }
    }

    private double POVDrive(int povVal, double vXVal, double vYVal) {
        double headingX = 0;
        double headingY = 0;

        switch (povVal) {
            case 0:
                headingY = 1;
                break;
            case 45:
                headingX = -1;
                headingY = 1;
                break;
            case 90:
                headingX = -1;
                break;
            case 135:
                headingX = -1;
                headingY = -1;
            case 180:
                headingY = -1;
                break;
            case 225:
                headingX = 1;
                headingY = -1;
                break;
            case 270:
                headingX = 1;
                break;
            case 315:
                headingX = 1;
                headingY = 1;
                break;
        }

        double isRedAlliance = swerve.isRedAlliance() ? -1 : 1;
        ChassisSpeeds povSpeeds =
                swerve.getTargetSpeeds(vYVal, vXVal, headingX * isRedAlliance, headingY * isRedAlliance);

        return povSpeeds.omegaRadiansPerSecond;
    }

    private double ReefLock(double vXVal, double vYVal) {
        return -1;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
