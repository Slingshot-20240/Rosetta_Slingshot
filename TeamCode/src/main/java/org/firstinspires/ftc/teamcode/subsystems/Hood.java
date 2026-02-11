package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Hood implements Subsystem {

    // ------------------ INSTANCES ------------------ //

    public static final Hood INSTANCE = new Hood();

    private Hood() {}

    private final ServoEx variableHood = new ServoEx("variableHood",-0.1);

    // ------------------ AUTONOMOUS COMMANDS ------------------ //

    public Command closeSide() {
        return new SetPosition(variableHood, 0.4).requires(this);
    }

    public Command farSide() {
        return new SetPosition(variableHood, 0.42).requires(this);
    }

    public Command setHoodPos(double hoodPosition) {
        return new SetPosition(variableHood, hoodPosition).requires(this);
    }

    // ------------------ TELEOP COMMANDS ------------------ //

    public Command instanceSetPos(double position) {
        return new InstantCommand(() -> variableHood.setPosition(position));
    }

}