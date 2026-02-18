package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Stoppernf implements Subsystem {
    public static final Stoppernf INSTANCE = new Stoppernf();
    private Stoppernf() {}

    public ServoEx blocker;


    public Command close() {
        return new InstantCommand(() -> blocker.getServo().setPosition(0.2));
    }
    public Command open() {
        return new InstantCommand(() -> blocker.getServo().setPosition(0.4));
    }

    @Override
    public void initialize() {
        blocker = new ServoEx("blocker");
    }

    @Override
    public void periodic() {}
}