package org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups;


import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Breakbeamnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

//AUTON SEQUENCES CLOSE
public class s extends SubsystemGroup {
    public static final s i = new s();
    private s() {
        super(
                Intakenf.INSTANCE, Transfernf.INSTANCE,
                Shooternf.INSTANCE, Hoodnf.INSTANCE
        );

    }

    /**
     * Waits until pathChain is at end
     * Opens blocker
     * Transfers up for shootTime seconds
     * Close blocker
     * @param shootTime time transfer shoots for
     * @param pathChain last pathChain of sequence
     * @return Sequential Group
     */
    public Command shootSequence(double shootTime, PathChain pathChain) {
        return new SequentialGroup(
                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),
                Transfernf.INSTANCE.open(),
                Transfernf.INSTANCE.on(),
                new Delay(shootTime)
        );
    }

    /**
     * Blocker opens
     * Transfers up for shootTime seconds
     * Blocker closes
     * @param shootTime
     * @return
     */
    public Command shoot(double shootTime) {
        return new SequentialGroup(
                Transfernf.INSTANCE.shootSet(),
                new Delay(shootTime)
        );
    }

    /**
     * Intake down, and on throughout
     * Blocker closed
     * Transfer on until first ball detected
     * @return
     */
    public Command intakeSequence() {
        return new ParallelGroup(
                Intakenf.INSTANCE.downAndOn(),

                new SequentialGroup(
                    Transfernf.INSTANCE.on(),
                    new WaitUntil(() -> Breakbeamnf.INSTANCE.getCount() == 1),
                    Transfernf.INSTANCE.off()
                )
        );
    }

    /**
     * Intake up, and on throughout
     * Transfer off
     * @return
     */
    public Command goScoreSequence() {
        return new SequentialGroup(
                Intakenf.INSTANCE.upAndOn(),
                Transfernf.INSTANCE.off()
        );
    }

    /**
     * Sets shooter speed
     * Sets hood angle to default 0.3
     * @param velocity
     * @return
     */
    public Command shooterState(double velocity) {
        return new ParallelGroup(
                Shooternf.INSTANCE.setShooterVel(velocity),
                Hoodnf.INSTANCE.setHoodPos(0.3)
        );
    }

    /**
     * Sets shooter speed
     * Sets hood angle to hoodPos
     * @param velocity
     * @param hoodPos
     * @return
     */
    public Command shooterState(double velocity, double hoodPos) {
        return new ParallelGroup(
                Shooternf.INSTANCE.setShooterVel(velocity),
                Hoodnf.INSTANCE.setHoodPos(hoodPos)
        );
    }


}