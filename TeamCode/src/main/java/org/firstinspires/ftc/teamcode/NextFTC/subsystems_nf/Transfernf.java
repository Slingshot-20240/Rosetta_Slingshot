package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfernf implements Subsystem {
    public static final Transfernf INSTANCE = new Transfernf();
    private Transfernf() {}

    public MotorEx frontTransfer;
    public ServoEx blocker;
    private final ControlSystem transferController = ControlSystem.builder()
            .velPid(1, 0, 0.001)
            .basicFF(0.01)
            .build();


    //Motor Commands
    public Command on() {
        return new ParallelGroup(
                new SetPower(frontTransfer, -1.0),
                open()
        ).addRequirements(frontTransfer, blocker);
    }

    public Command idle() {
        return new SequentialGroup(
                new SetPower(frontTransfer,0),
                close()
        ).addRequirements(frontTransfer, blocker);
    }

    //Blocker Commands
    public Command close() {
        return new SetPosition(blocker, 0.4).requires(this);
    }
    public Command open() {
        return new SetPosition(blocker, 0.2).requires(this);
    }


    @Override
    public void initialize() {
        frontTransfer = new MotorEx("transferF");
        blocker = new ServoEx("blocker");
    }

    @Override
    public void periodic() {}
}