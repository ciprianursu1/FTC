package org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueShared {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;


    public BlueShared(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 28.000),

                                new Pose(46.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(127.0), Math.toRadians(263.5))
                .build();
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.000, 60.000),

                                new Pose(46.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(263.5), Math.toRadians(116.0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.000, 60.000),

                                new Pose(46.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116.0), Math.toRadians(180.0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.000, 60.000),

                                new Pose(18.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 60.000),

                                new Pose(46.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(116.0))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.000, 60.000),

                                new Pose(46.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116.0), Math.toRadians(180.0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.000, 84.000),

                                new Pose(18.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 84.000),

                                new Pose(46.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(116.0))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.000, 60.000),

                                new Pose(34.000, 34.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116.0), Math.toRadians(180.0))
                .build();

    }
}