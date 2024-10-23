package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

public class MMBatteryTest extends MMOpMode {

    public MMBatteryTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    //notice that its okay to leave it empty.
    @Override
    public void onInit() {}

    @Override
    public void run() {
        super.run();
        telemetry.addData(
                "Battery",
                MMRobot.getInstance().mmSystems.battery.getPercentage()
        );
        telemetry.update();
    }
}
