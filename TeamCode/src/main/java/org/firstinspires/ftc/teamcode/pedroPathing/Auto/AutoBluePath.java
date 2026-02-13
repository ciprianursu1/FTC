package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PathOnlyAuto", group = "Test")
public class AutoBluePath extends OpMode {

    public Follower follower;
    private Paths paths;

    private boolean pathStarted = false;
    private int stage = 0;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        // Starting pose (first pose in your system)
        follower.setStartingPose(new Pose(24.000, 126.000, Math.toRadians(324)));

        paths = new Paths(follower);
    }

    @Override
    public void loop() {

        follower.update();

        switch (stage) {

            // Move to shoot
            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.Path9, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage++;
                }
                break;

            // Stack approach
            case 1:
                if (!pathStarted) {
                    follower.followPath(paths.Path1, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage++;
                }
                break;

            // Stack sweep
            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Path2, 0.5, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage++;
                }
                break;

            // Return
            case 3:
                if (!pathStarted) {
                    follower.followPath(paths.Path4, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage++;
                }
                break;

            // Second approach
            case 4:
                if (!pathStarted) {
                    follower.followPath(paths.Path5, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage++;
                }
                break;

            // Second sweep
            case 5:
                if (!pathStarted) {
                    follower.followPath(paths.Path6, 0.5, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage++;
                }
                break;

            // Return again
            case 6:
                if (!pathStarted) {
                    follower.followPath(paths.Path7, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage++;
                }
                break;

            // Park
            case 7:
                if (!pathStarted) {
                    follower.followPath(paths.Path8, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    stage++;
                }
                break;

            case 8:
                requestOpModeStop();
                break;
        }
    }

    // ===================== PATH DEFINITIONS =====================
    public static class Paths {

        public PathChain Path9;
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
                            new Pose(24.000, 126.000),
                            new Pose(54.000, 102.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(324),
                            Math.toRadians(324))
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 102.000),
                            new Pose(42.000, 84.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(324),
                            Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.000, 84.000),
                            new Pose(18.000, 84.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.000, 84.000),
                            new Pose(54.000, 102.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(324))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 102.000),
                            new Pose(42.000, 60.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(324),
                            Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.000, 60.000),
                            new Pose(18.000, 60.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.000, 60.000),
                            new Pose(54.000, 102.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(324))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 102.000),
                            new Pose(13.000, 59.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(324),
                            Math.toRadians(150))
                    .build();
        }
    }
}
