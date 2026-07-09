package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueMiddle {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;


    public BlueMiddle(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.000, 136.000),

                                new Pose(62.000, 124.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(266.6))
                .build();
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(62.000, 124.000),

                                new Pose(62.000, 124.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(266.6), Math.toRadians(95.0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(62.000, 124.000),

                                new Pose(44.000, 108.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(95.0), Math.toRadians(180.0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 108.000),

                                new Pose(16.000, 108.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 108.000),

                                new Pose(62.000, 124.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(95.0))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(62.000, 124.000),

                                new Pose(44.000, 180.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(95.0), Math.toRadians(180.0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 180.000),

                                new Pose(16.000, 180.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 180.000),

                                new Pose(62.000, 124.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(95.0))
                .build();
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(62.000, 124.000),

                                new Pose(45.000, 108.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(95.0), Math.toRadians(270.0))
                .build();

    }
}
