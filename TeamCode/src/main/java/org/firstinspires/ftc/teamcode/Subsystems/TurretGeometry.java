package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.geometry.Pose;

public final class TurretGeometry {
    private TurretGeometry() {
    }

    public static double turretAngleToPoint(
            Pose robotPose,
            double pivotOffsetX,
            double pivotOffsetY,
            double targetX,
            double targetY) {
        double heading = robotPose.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double pivotX = robotPose.getX()
                + (pivotOffsetX * cos - pivotOffsetY * sin) * RobotConfig.INCHES_PER_METER;
        double pivotY = robotPose.getY()
                + (pivotOffsetX * sin + pivotOffsetY * cos) * RobotConfig.INCHES_PER_METER;
        double fieldAngle = Math.atan2(targetY - pivotY, targetX - pivotX);
        return wrapAngle(Math.toDegrees(fieldAngle - heading));
    }

    public static double mountedTurretAngleToPoint(
            Pose robotPose,
            double pivotOffsetX,
            double pivotOffsetY,
            double mountedOffsetY,
            double targetX,
            double targetY) {
        double heading = robotPose.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double pivotX = robotPose.getX()
                + (pivotOffsetX * cos - pivotOffsetY * sin) * RobotConfig.INCHES_PER_METER;
        double pivotY = robotPose.getY()
                + (pivotOffsetX * sin + pivotOffsetY * cos) * RobotConfig.INCHES_PER_METER;
        double targetLocalX = (targetX - pivotX) * cos + (targetY - pivotY) * sin;
        double targetLocalY = -(targetX - pivotX) * sin + (targetY - pivotY) * cos;
        double distance = Math.hypot(targetLocalX, targetLocalY);
        double lateralOffset = mountedOffsetY * RobotConfig.INCHES_PER_METER;

        if (distance <= Math.abs(lateralOffset)) {
            return turretAngleToPoint(robotPose, pivotOffsetX, pivotOffsetY, targetX, targetY);
        }

        return wrapAngle(Math.toDegrees(Math.atan2(targetLocalY, targetLocalX) - Math.asin(lateralOffset / distance)));
    }

    private static double wrapAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle - 180;
    }
}
