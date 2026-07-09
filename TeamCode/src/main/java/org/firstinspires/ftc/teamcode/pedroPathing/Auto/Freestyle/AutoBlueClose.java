package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Freestyle;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.TurretGeometry;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.BlueClose;

@Autonomous(name = "AutoBlueClose", group = "AutoTimisoara")
public class AutoBlueClose extends FreestyleAutoBase {
    private BlueClose paths;
    private boolean requestedShoot = false;
    private boolean resetMotifTimer = true;
    private PathChain parkPath = null;
    private final ElapsedTime motifFallbackTimer = new ElapsedTime();

    @Override
    protected Pose getStartPose() {
        return new Pose(26.0, 276.0, Math.toRadians(324.0));
    }

    @Override
    protected void initPaths() {
        paths = new BlueClose(follower);
    }

    @Override
    protected void runPathState() {
        if(timer.seconds() > RobotConfig.AUTO_MAX_TIME) {
            if(pathState != 9) {
                if(parkPath == null){
                    parkPath = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            pose,

                                            new Pose(44.000, 204.000)
                                    )
                            )
                            .setLinearHeadingInterpolation(pose.getHeading(), Math.toRadians(180.0))
                            .build();
                    pathStarted = false;
                }
                followPath(parkPath,9, RobotConfig.AUTO_PATH_POWER);
                robot.artifactHandler.requestShoot(false);
            } else {
                requestOpModeStop();
            }
            return;
        }
        switch (pathState) {
            case 0:
                robot.artifactHandler.setTurretTargetAngleOverride(TurretGeometry.mountedTurretAngleToPoint(
                        pose,
                        RobotConfig.TURRET_OFFSET_X_METERS,
                        RobotConfig.TURRET_OFFSET_Y_METERS,
                        RobotConfig.LIMELIGHT_OFFSET_Y_METERS,
                        RobotConfig.MOTIF_X,
                        RobotConfig.MOTIF_Y
                ));
                followPath(paths.Path1, 1,RobotConfig.AUTO_PATH_POWER);
                break;
            case 1:
                if(resetMotifTimer){
                    motifFallbackTimer.reset();
                    resetMotifTimer = false;
                }
                if(motifFallbackTimer.seconds() > RobotConfig.MOTIF_FALLBACK_TIME){
                    robot.artifactHandler.setMotif(RobotConfig.DEFAULT_MOTIF);
                    robot.artifactHandler.clearTurretTargetAngleOverride();
                    robot.artifactHandler.setTargetX(RobotConfig.BLUE_PRIMARY_TARGET_X);
                    robot.artifactHandler.setTargetY(RobotConfig.PRIMARY_TARGET_Y);
                    robot.artifactHandler.turnOnIntake(true,false);
                    followPath(paths.Path2, 2,RobotConfig.AUTO_PATH_POWER);
                    break;
                }
                if(limelight.isFreshData()){
                    switch(limelight.getTagID()){
                        case 21:
                            robot.artifactHandler.setMotif(new int[]{1,2,2});
                            robot.artifactHandler.clearTurretTargetAngleOverride();
                            robot.artifactHandler.setTargetX(RobotConfig.BLUE_PRIMARY_TARGET_X);
                            robot.artifactHandler.setTargetY(RobotConfig.PRIMARY_TARGET_Y);
                            robot.artifactHandler.turnOnIntake(true,false);
                            followPath(paths.Path2, 2,RobotConfig.AUTO_PATH_POWER);
                            break;
                        case 22:
                            robot.artifactHandler.setMotif(new int[]{2,1,2});
                            robot.artifactHandler.clearTurretTargetAngleOverride();
                            robot.artifactHandler.setTargetX(RobotConfig.BLUE_PRIMARY_TARGET_X);
                            robot.artifactHandler.setTargetY(RobotConfig.PRIMARY_TARGET_Y);
                            robot.artifactHandler.turnOnIntake(true,false);
                            followPath(paths.Path2, 2,RobotConfig.AUTO_PATH_POWER);
                            break;
                        case 23:
                            robot.artifactHandler.setMotif(new int[]{2,2,1});
                            robot.artifactHandler.clearTurretTargetAngleOverride();
                            robot.artifactHandler.setTargetX(RobotConfig.BLUE_PRIMARY_TARGET_X);
                            robot.artifactHandler.setTargetY(RobotConfig.PRIMARY_TARGET_Y);
                            robot.artifactHandler.turnOnIntake(true,false);
                            followPath(paths.Path2, 2,RobotConfig.AUTO_PATH_POWER);
                            break;
                    }
                }
                break;
            case 2:
                if(!requestedShoot) {
                    robot.artifactHandler.requestShoot(true);
                    requestedShoot = true;
                }
                if(!robot.artifactHandler.getOuttakeState()) {
                    robot.artifactHandler.requestShoot(false);
                    requestedShoot = false;
                    followPath(paths.Path3, 3,RobotConfig.AUTO_PATH_POWER);
                }
                break;
            case 3:
                followPath(paths.Path4, 4,RobotConfig.AUTO_INTAKE_PATH_POWER);
                break;
            case 4:
                followPath(paths.Path5, 5,RobotConfig.AUTO_PATH_POWER);
                break;
            case 5:
                if(!requestedShoot) {
                    robot.artifactHandler.requestShoot(true);
                    requestedShoot = true;
                }
                if(!robot.artifactHandler.getOuttakeState()) {
                    robot.artifactHandler.requestShoot(false);
                    requestedShoot = false;
                    followPath(paths.Path6, 6,RobotConfig.AUTO_PATH_POWER);
                }
                break;
            case 6:
                followPath(paths.Path7, 7,RobotConfig.AUTO_INTAKE_PATH_POWER);
                break;
            case 7:
                followPath(paths.Path8, 8,RobotConfig.AUTO_PATH_POWER);
                break;
            case 8:
                if(!requestedShoot) {
                    robot.artifactHandler.requestShoot(true);
                    requestedShoot = true;
                }
                if(!robot.artifactHandler.getOuttakeState()) {
                    robot.artifactHandler.requestShoot(false);
                    requestedShoot = false;
                    followPath(paths.Path9, 9,RobotConfig.AUTO_PATH_POWER);
                }
                break;
            case 9:
                requestOpModeStop();
                break;

        }
    }
}
