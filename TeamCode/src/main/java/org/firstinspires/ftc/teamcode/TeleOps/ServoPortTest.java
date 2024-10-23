package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
@TeleOp
public class ServoPortTest extends MMOpMode {
    public ServoPortTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    CuttleServo servo0;
    CuttleServo servo1;
    CuttleServo servo2;
    CuttleServo servo3;
    CuttleServo servo4;
    CuttleServo servo5;
    Servo s555;


    @Override
    public void onInit() {
        s555 = hardwareMap.servo.get("s55");
        servo0 = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 0);
        servo1 = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 1);
        servo2 = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 2);
        servo3 = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 3);
        servo4 = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 4);
//        servo5 = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 5);
    }

    @Override
    public void run() {
        super.run();
        s555.setPosition(gamepad1.left_trigger);
//        servo0.setPosition(gamepad1.left_trigger);
//        servo1.setPosition(gamepad1.left_trigger);
//        servo2.setPosition(gamepad1.left_trigger);
//        servo3.setPosition(gamepad1.left_trigger);
//        servo4.setPosition(gamepad1.left_trigger);
//        servo5.setPosition(gamepad1.left_trigger);

    }
}
