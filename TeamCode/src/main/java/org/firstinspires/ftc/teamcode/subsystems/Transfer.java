package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.paths.PathChain;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() {}

    public MotorEx transfer;
    public ServoEx blocker;
    private final ControlSystem transferController = ControlSystem.builder()
            .velPid(1, 0, 0.001)
            .basicFF(0.01)
            .build();


    //Motor Commands
    public Command on() {
        return new SetPower(transfer, -1.0);
    }

    public Command off() {
        return new SetPower(transfer, 0.0);
    }

    //Blocker Commands
    public Command close() {
        return new InstantCommand(() -> blocker.getServo().setPosition(0.2));
    }
    public Command open() {
        return new InstantCommand(() -> blocker.getServo().setPosition(0.4));
    }



    @Override
    public void initialize() {
        transfer = new MotorEx("transfer");
        blocker = new ServoEx("blocker");
    }

    @Override
    public void periodic() {}
}