package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "AutoBlueFar_PATH_ONLY", group = "Test")
public class Auto extends OpMode {

    Follower follower;
    Paths paths;

    int stage = 0;
    boolean pathStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Matches Path9 start
        follower.setStartingPose(new Pose(57, 9, Math.toRadians(90)));

        paths = new Paths(follower);

        stage = 0;
        pathStarted = false;
    }

    @Override
    public void loop() {

        follower.update();

        switch (stage) {

            case 0:
                stage = 1;
                break;

            case 1:
                if (!pathStarted) {
                    follower.followPath(paths.Path9, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 2;
                }
                break;

            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Path3, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 3;
                }
                break;

            case 3:
                if (!pathStarted) {
                    follower.followPath(paths.Path1, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 4;
                }
                break;

            case 4:
                if (!pathStarted) {
                    follower.followPath(paths.Path2, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 5;
                }
                break;

            case 5:
                if (!pathStarted) {
                    follower.followPath(paths.Path4, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    follower.followPath(paths.Path5, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 7;
                }
                break;

            case 7:
                if (!pathStarted) {
                    follower.followPath(paths.Path6, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 8;
                }
                break;

            case 8:
                if (!pathStarted) {
                    follower.followPath(paths.Path7, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 9;
                }
                break;

            case 9:
                if (!pathStarted) {
                    follower.followPath(paths.Path8, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    stage = 10;
                }
                break;

            case 10:
                requestOpModeStop();
                break;
        }
    }

    public static class Paths {

        public PathChain Path9;
        public PathChain Path3;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 9.000),
                            new Pose(57.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 12.000),
                            new Pose(57.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(299))
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 21.000),
                            new Pose(42.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.000, 36.000),
                            new Pose(9.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.000, 36.000),
                            new Pose(57.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(299))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 21.000),
                            new Pose(12.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(200))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.000, 21.000),
                            new Pose(12.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.000, 12.000),
                            new Pose(57.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(299))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 21.000),
                            new Pose(39.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(180))
                    .build();
        }
    }
}
