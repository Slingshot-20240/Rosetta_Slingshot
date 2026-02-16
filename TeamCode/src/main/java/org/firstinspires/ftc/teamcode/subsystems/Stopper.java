package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {
    public final Servo stopper;

    public Stopper(HardwareMap hardwareMap) {
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    public Stopper(Servo stopper) {
        this.stopper = stopper;
    }
// ------------------------------------------------------------------

    public void stop() {
        stopper.setPosition(1);
    }

    public void release() {
        stopper.setPosition(0);
    }
}

