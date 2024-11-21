package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class LinearIntakeArmTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    boolean open = false;

    public LinearIntakeArmTeleOp(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance.mmSystems.initRobotSystems();
        ButtonReader AButtonCondition = new ButtonReader(
                () -> robotInstance.mmSystems.gamepadEx1.gamepad.a
        );

        if (AButtonCondition.isDown())
        {
            if (!open){
                RobotCommands.IntakeCommand();

            }
            else {
                RobotCommands.IntakeDoneCommand();
            }
        }

        waitForStart();

    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Joystick Position: ", robotInstance.mmSystems.linearIntake.getPosition());
        telemetry.update();
    }

}