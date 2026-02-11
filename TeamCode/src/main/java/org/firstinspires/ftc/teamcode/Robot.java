package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;

public class Robot {

    // ------------------ CONFIG ------------------ //

    // right - expansion
    // left - control
    // front - 0
    // back - 1

    // servo variableHood    control hub _
    // servo transferB       control hub _

    // servo park            expansion hub _

    // motor outtakeLeft     control hub _
    // motor outtakeRight    control hub _

    // motor transferF       expansion hub port _
    // motor intake          expansion hub port _

    // led                   control hub _-_

    // ------------------ MECHANISMS ------------------ //

    public final IMU imu;

    // ------------------ SENSORS ------------------ //

    public GoBildaPinpointDriver driver;

    // ------------------ MISC ------------------ //

    public GamepadMapping controls;

    public Robot(HardwareMap hardwareMap, GamepadMapping controls) {
        this.controls = controls;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }
}
