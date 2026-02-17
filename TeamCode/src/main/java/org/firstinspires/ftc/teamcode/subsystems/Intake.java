package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public final DcMotorEx intake1;
    public final DcMotorEx intake2;

    public final DigitalChannel beamBreakLow, beamBreakMid, beamBreakHigh;

    private int count = 0;
    private boolean lastDetected = false;


    public Intake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        beamBreakLow = hardwareMap.get(DigitalChannel.class, "bbl");
        beamBreakMid = hardwareMap.get(DigitalChannel.class, "bbm");
        beamBreakHigh = hardwareMap.get(DigitalChannel.class, "bbh");

        beamBreakLow.setMode(DigitalChannel.Mode.INPUT);
        beamBreakMid.setMode(DigitalChannel.Mode.INPUT);
        beamBreakHigh.setMode(DigitalChannel.Mode.INPUT);
    }

    // constructor for JUnit
    public Intake(DcMotorEx intake1, DcMotorEx intake2,
                  DigitalChannel bb1, DigitalChannel bb2, DigitalChannel bb3) {
        this.intake2 = intake2;
        this.intake1 = intake1;

        beamBreakLow = bb1;
        beamBreakMid = bb2;
        beamBreakHigh = bb3;
    }

//-------------------------------------------------------------------------------

    public void intakeTransferOn() {
        intake2.setPower(1);
        intake1.setPower(1);
    }

    public void intakeTransferOff() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    public void intakeTransferReverse() {
        intake2.setPower(-1);
        intake1.setPower(-1);
    }

    public int getCount() {
        boolean detected = beamBreakLow.getState();
        if (detected && !lastDetected) {
            count++;
        }

        lastDetected = detected;

        return count;
    }

}
