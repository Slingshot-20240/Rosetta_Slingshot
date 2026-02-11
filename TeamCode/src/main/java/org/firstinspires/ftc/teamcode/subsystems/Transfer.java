package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {

    // ------------------ INSTANCES ------------------ //

    public static final Transfer INSTANCE = new Transfer();

    private Transfer() {}

    public MotorEx frontTransfer;

    public ServoEx blocker;

    private final ControlSystem transferController = ControlSystem.builder()
            .velPid(1, 0, 0.001)
            .basicFF(0.01)
            .build();

    // ------------------ AUTONOMOUS COMMANDS ------------------ //

    // --- Motor --- //
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

    // --- Blocker --- //
    public Command close() {
        return new SetPosition(blocker, 0.4).requires(this);
    }
    public Command open() {
        return new SetPosition(blocker, 0.2).requires(this);
    }

    public Command shoot() {
        return new SequentialGroup(
                open(),
                on()
        );
    }

    @Override
    public void initialize() {
        frontTransfer = new MotorEx("transferF");
        blocker = new ServoEx("blocker");
    }

    @Override
    public void periodic() {}

    // ------------------ TELEOP COMMANDS ------------------ //

    public Command instanceTransferUp() {
        return new InstantCommand(() -> frontTransfer.getMotor().setPower(-1));
    }

    public Command instanceTransferIdle() {
        return new InstantCommand(() -> frontTransfer.getMotor().setPower(0));
    }

    public Command instanceTransferHotDog() {
        return new InstantCommand(() -> frontTransfer.getMotor().setPower(0.35));
    }

    public Command instanceBlockerClose() {
        return new InstantCommand(() -> blocker.getServo().setPosition(1));
    }

    public Command instanceBlockerOpen() {
        return new InstantCommand(() -> blocker.getServo().setPosition(0));
    }
}