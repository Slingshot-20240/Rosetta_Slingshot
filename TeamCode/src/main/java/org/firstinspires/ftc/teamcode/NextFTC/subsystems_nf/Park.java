package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Park implements Subsystem {

    // ------------------ INSTANCES ------------------ //

    public static final Park INSTANCE = new Park();

    private Park() {
    }

    private final ServoEx parkServo = new ServoEx("park");

    // ------------------ COMMANDS ------------------ //

    public Command instanceTilt() {
        return new InstantCommand(() -> parkServo.getServo().setPosition(0.06));
    }

    public Command instanceUnTilt() {
        return new InstantCommand(() -> parkServo.getServo().setPosition(0.49));
    }
}
