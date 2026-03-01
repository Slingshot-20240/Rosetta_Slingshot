package org.firstinspires.ftc.teamcode.NextFTC.autonomous.close;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.s;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Stoppernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Red 12 Close")
public class RedClose15 extends NextFTCOpMode {
    public RedClose15() {
        addComponents(
                new SubsystemComponent(
                        f.i, s.i,
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Stoppernf.INSTANCE,
                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads_grabSet2;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain gateSet3;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain grabHp;
    public PathChain scoreHp;

    public Pose scorePose = new Pose(92,88);

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads_grabSet2 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.2, 119),
                                //TODO - try working with increasing x below, this makes it not go as far left for shooting
                                // position. This means lower shooter speed and make hood lower for more arc
                                new Pose(55.400, 76.100),
                                new Pose(103.389, 85.000),
                                new Pose(129, 83)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.1,
                                        HeadingInterpolator.constant(Math.toRadians(36))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.1,
                                        0.45,
                                        HeadingInterpolator.facingPoint(141.5, 141.5)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.45,
                                        1.0,
                                        HeadingInterpolator.linearFromPoint(
                                                //TODO - whatever current pose is, not 45 degrees hardcoded
                                                () -> follower().getHeading(),
                                                Math.toRadians(0),
                                                0.995
                                        )
                                )
                        )
                )
                .addParametricCallback(0.35, () -> follower().setMaxPower(0.35))
                .addParametricCallback(0.45, () -> follower().setMaxPower(1.0))

                .build();

        scoreSet2 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130.000, 83.000),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))

                .build();

        grabSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88.941, 53.686),
                                new Pose(96.582, 59.502),
                                new Pose(134, 58)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.55,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(49),
                                                Math.toRadians(0)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.55,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )

                .build();

        gateSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134, 58),
                                new Pose(105.151, 58.012),
                                new Pose(128.000, 69.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        scoreSet3 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 69.000),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
                .build();

        grabSet4 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(86.478, 29.598),
                                new Pose(98.351, 36.067),
                                new Pose(135, 35.5)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(49),
                                                Math.toRadians(0)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )

                .build();

        scoreSet4 = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135, 35.5),
                                scorePose
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.1,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.1,
                                        0.8,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1.0,
                                        HeadingInterpolator.facingPoint(new Pose(139,141.5))
                                )
                        )
                )
                .build();

        grabHp = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(130.000, 58.000),
                                new Pose(132.000, 12.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreHp = PedroComponent.follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.000, 12.000),
                                new Pose(88, 110)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.9,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.9,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(30))
                                )
                        )
                )
                .build();



    }

    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.37)
        );

    }

    private Command autonomous() {
        return new SequentialGroup(

                new ParallelDeadlineGroup(
                        f.i.follow(scorePreloads_grabSet2, "green"),

                        Intakenf.INSTANCE.in(),
                        //TODO - if you need more time for flywheel to spinup,
                        // slow down the speed even more of the path when its supposed to be shooting!
                        s.i.shooterState(-1190,0.37),
                        s.i.shootAt(follower(), scorePreloads_grabSet2, 2, 0.35)
                ),


                //Set 2
                new ParallelDeadlineGroup(
                        s.i.shootSequence(scoreSet2, 0.8),

                        f.i.follow(scoreSet2, "green"),
                        s.i.shooterState(-1250,0.35)
                ),

                //Set 3
                new ParallelDeadlineGroup(
                        s.i.shootSequence(scoreSet3, 0.8),

                        new SequentialGroup(
                                f.i.follow(grabSet3, "red"),
                                f.i.follow(scoreSet3, "green")
                        )
                ),

                //Set 4
                new ParallelDeadlineGroup(
                        s.i.shootSequence(scoreSet4, 0.8),

                        new SequentialGroup(
                                f.i.follow(grabSet4, "red"),
                                f.i.follow(scoreSet4, "green")
                        )
                ),

                //Hp
                new ParallelDeadlineGroup(
                        s.i.shootSequence(scoreHp, 0.8),

                        new SequentialGroup(
                                f.i.follow(grabHp, "red"),
                                f.i.follow(scoreHp, "green")
                        )
                )



        );
    }


    @Override
    public void onInit() {
        buildPaths();
        init_bot().schedule();
        Shooternf.INSTANCE.disable();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
        Shooternf.INSTANCE.enable();
    }

    @Override
    public void onStop() {
        PoseStorage.startingPose = follower().getPose();
    }
}