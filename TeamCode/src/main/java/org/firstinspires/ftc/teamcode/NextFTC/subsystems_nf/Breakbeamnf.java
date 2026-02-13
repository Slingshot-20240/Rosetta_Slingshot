package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Breakbeamnf implements Subsystem {
    public static final Breakbeamnf INSTANCE = new Breakbeamnf();
    private Breakbeamnf() {}

    private DigitalChannel bblow;
    private DigitalChannel bbmid;
    private DigitalChannel bbhigh;


    @Override
    public void initialize() {
        bblow = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "bbl");
        bbmid = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "bbm");
        bbhigh = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "bbh");

        bblow.setMode(DigitalChannel.Mode.INPUT);
        bbmid.setMode(DigitalChannel.Mode.INPUT);
        bbhigh.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public void periodic() {

    }
}