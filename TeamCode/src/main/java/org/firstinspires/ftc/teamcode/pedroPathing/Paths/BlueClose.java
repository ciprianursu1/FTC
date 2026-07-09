package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClose {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;

    public BlueClose(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(26.000, 276.000),

                                new Pose(60.000, 226.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(324.0), Math.toRadians(259.2))
                .build();
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 226.000),

                                new Pose(60.000, 226.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(259.2), Math.toRadians(311.5))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 226.000),

                                new Pose(44.000, 228.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(311.5), Math.toRadians(180.0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 228.000),

                                new Pose(17.000, 228.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.000, 228.000),

                                new Pose(60.000, 226.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(311.5))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 226.000),

                                new Pose(44.000, 204.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(311.5), Math.toRadians(180.0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 204.000),

                                new Pose(9.000, 203.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 203.000),

                                new Pose(60.000, 226.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(311.5))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 226.000),

                                new Pose(44.000, 204.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(311.5), Math.toRadians(180.0))
                .build();

    }
}
