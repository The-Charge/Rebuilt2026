package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.Logger;


// Get the FieldZone (Blue, Neutral, Red) the Robot is in based on its Pose
public enum FieldZone {
    BLUE,
    NEUTRAL,
    RED;

    public static FieldZone fromRobotX(Distance robotXPose) {
        if (robotXPose == null) {
            Logger.reportWarning("Cannot determine FieldZone from null robot x", true);
            return NEUTRAL;
        }

        if (robotXPose.lt(FieldConstants.blueZoneEdge)) {
            return BLUE;
        } else if (robotXPose.lte(FieldConstants.redZoneEdge)) {
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
