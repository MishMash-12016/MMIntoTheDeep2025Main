package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class BatteryTest extends MMOpMode {

    MMBattery mmBattery;

    public BatteryTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        mmBattery = new MMBattery(hardwareMap);
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Voltage", mmBattery.getVoltage());
        telemetry.update();
    }
}
