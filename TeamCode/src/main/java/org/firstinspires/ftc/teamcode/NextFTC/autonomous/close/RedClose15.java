package org.firstinspires.ftc.teamcode.NextFTC.autonomous.close;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.s;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
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
                        Shooternf.INSTANCE, Transfernf.INSTANCE,
                        Lednf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;
    public PathChain grabSet2;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain scoreSet4;
    public PathChain grabSet4;
    public PathChain grabHp;
    public PathChain scoreHp;

    public Pose scorePose = new Pose(87,84);

    public void buildPaths() {
        follower().setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));

        scorePreloads = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.406, 118.970),

                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        grabSet2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                scorePose,

                                new Pose(129.000, 84.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreSet2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.000, 84.000),

                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed()
                .build();

        grabSet3 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.400, 54.000),
                                new Pose(94.000, 59.100),
                                new Pose(134.000, 58.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                //GATE CODE
                .addPath(
                        new BezierCurve(
                                new Pose(134.000, 58.600),
                                new Pose(108.432, 58.197),
                                new Pose(128.000, 69.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();


        scoreSet3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 69.000),

                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed()
                .build();

        grabSet4 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(87.214, 26.366),
                                new Pose(89.201, 36.446),
                                new Pose(134.100, 35.306)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreSet4 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.100, 35.306),

                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed()

                .build();

        grabHp = follower().pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(135.826, 49.127),
                                new Pose(135.311, 9.851)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreHp = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.311, 9.851),

                                new Pose(86.410, 108.728)
                        )
                ).setTangentHeadingInterpolation().setReversed()
                .build();



    }

    private Command init_bot() {
        return new ParallelGroup(
                Hoodnf.INSTANCE.setHoodPos(0.35),
                Intakenf.INSTANCE.down()
        );

    }

    private Command autonomous() {
        return new SequentialGroup(

                //Score Preloads
                new ParallelGroup(
                        f.i.follow(scorePreloads),

                        s.i.goScoreSequence(),
                        s.i.shooterState(1250,0.35),
                        s.i.shootSequence(scorePreloads, 0.4)
                ),

                //Set 2
                new ParallelGroup(
                        f.i.follow(grabSet2),
                        s.i.intakeSequence()
                ),
                new ParallelGroup(
                        f.i.follow(scoreSet2),

                        s.i.goScoreSequence(),
                        s.i.shooterState(1250,0.35), //if the robot is stuck try taking out this lien
                        s.i.shootSequence(scoreSet2, 0.4)
                ),

                //Set 3
                new ParallelGroup(
                        f.i.follow(grabSet3),
                        s.i.intakeSequence()
                ),
                new ParallelGroup(
                        f.i.follow(scoreSet3),

                        s.i.goScoreSequence(),
                        s.i.shooterState(1250,0.35), //if the robot is stuck try taking out this lien
                        s.i.shootSequence(scoreSet3, 0.4)
                ),

                //Set 4
                new ParallelGroup(
                        f.i.follow(grabSet4),
                        s.i.intakeSequence()
                ),
                new ParallelGroup(
                        f.i.follow(scoreSet4),

                        s.i.goScoreSequence(),
                        s.i.shooterState(1250,0.35), //if the robot is stuck try taking out this lien
                        s.i.shootSequence(scoreSet4, 0.4)
                ),

                //Human Player
                new ParallelGroup(
                        f.i.follow(grabHp),
                        s.i.intakeSequence()
                ),
                new ParallelGroup(
                        f.i.follow(scoreHp),

                        s.i.goScoreSequence(),
                        s.i.shooterState(1250,0.35), //if the robot is stuck try taking out this lien
                        s.i.shootSequence(scoreHp, 0.4)
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