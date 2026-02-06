package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

public enum FieldZone {
    BLUE,
    NEUTRAL,
    RED;

    public static FieldZone fromRobotX(Distance robotXPose) {
        if (robotXPose == null) {
            Logger.reportWarning("Cannot determine FieldZone from null robot x", true);
            return NEUTRAL;
        }

        double xInches = robotXPose.abs(Inches);
        /*
         * Numbers from field manual:
         * Blue alliance zone is from [0, 182.11)
         * Neutral zone is from [182.11, 469.11]
         * Red alliance zone is from (469.11, 651.22]
         */
        if (xInches < 182.11) {
            return BLUE;
        } else if (xInches <= 469.11) {
            return NEUTRAL;
        } else {
            return RED;
        }
    }

    public static FieldZone fromRobotPose(Pose2d robotPose) {
        if (robotPose == null) {
            Logger.reportWarning("Cannot determine FieldZone from null robot pose", true);
            return NEUTRAL;
        }

        return fromRobotX(robotPose.getMeasureX());
    }
}
