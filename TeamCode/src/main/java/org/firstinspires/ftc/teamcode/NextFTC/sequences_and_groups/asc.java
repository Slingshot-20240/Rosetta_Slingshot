package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;


import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Transfernf;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

//AUTON SEQUENCES CLOSE
public class asc extends SubsystemGroup {
    public static final asc i = new asc();
    private asc() {
        super(
                Intake.INSTANCE, Transfernf.INSTANCE,
                Shooter.INSTANCE, Hood.INSTANCE,
                Led.INSTANCE
        );

    }



    /**
     * Transfer on
     * Wait for time
     * @param time
     */
    public final Command transferUpFor(double time) {
        return new ParallelGroup(
                Transfernf.INSTANCE.on(),
                new Delay(time),
                Led.INSTANCE.yellow
        );
    }

    /**
     * Hotdog until last path of pathchain is at parametric end
     * Then LED green, and transfer up for transferTime
     * @param lastPathChain
     * @param transferTime
     */
    public final Command transferSequence(PathChain lastPathChain, double transferTime) {
        return new SequentialGroup(
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> lastPathChain.lastPath().isAtParametricEnd()),
                transferUpFor(transferTime)

        );
    }

    public final Command transferSequenceDistance(PathChain pathChain, double transferTime, double proximity) {
        return new SequentialGroup(
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() < proximity),
                transferUpFor(transferTime)
        );
    }
    public final Command transferSequenceDistance(PathChain pathChain, double transferTime, double proximity, double spinUp) {
        return new SequentialGroup(
                new Delay(spinUp),
                Transfernf.INSTANCE.hotdog(),
                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() < proximity),
                transferUpFor(transferTime)
        );
    }

    public final Command baseState(double shooterVel) {
        return new ParallelGroup(
                Intake.INSTANCE.in(),
                Shooter.INSTANCE.setShooterVel(shooterVel),
                //used to be 0.33
                Hood.INSTANCE.setHoodPos(0.35)
        );
    }
    public final Command baseState(double shooterVel, double hoodPos) {
        return new ParallelGroup(
                Intake.INSTANCE.in(),
                Shooter.INSTANCE.setShooterVel(shooterVel),
                Hood.INSTANCE.setHoodPos(hoodPos)
        );
    }


}