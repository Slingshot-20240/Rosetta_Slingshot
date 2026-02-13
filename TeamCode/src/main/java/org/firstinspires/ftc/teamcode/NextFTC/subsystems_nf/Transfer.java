package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
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
        return new InstantCommand(() -> transfer.getMotor().setPower(-1.0));
    }
    public Command off() {
        return new InstantCommand(() -> transfer.getMotor().setPower(0.0));
    }

//Blocker Commands
    public Command close() {
        return new InstantCommand(() -> blocker.getServo().setPosition(0.2));
    }
    public Command open() {
        return new InstantCommand(() -> blocker.getServo().setPosition(0.4));
    }

    public Command shootSet() {
        return new SequentialGroup(
                open(),
                on()
        );
    }

    @Override
    public void initialize() {
        transfer = new MotorEx("transfer");
        blocker = new ServoEx("blocker");
    }

    @Override
    public void periodic() {}
}