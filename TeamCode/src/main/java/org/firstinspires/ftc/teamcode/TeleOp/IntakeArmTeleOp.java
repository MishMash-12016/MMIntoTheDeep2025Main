package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class IntakeArmTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    public IntakeArmTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance.mmSystems.initRobotSystems();

        Trigger leftTrigger = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
        );

        Trigger rightTrigger = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
        );


        leftTrigger.whenActive(
                        robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.up)
                );


//                new SequentialCommandGroup(
//                        robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.down),
//                        robotInstance.mmSystems.intakEndUnit.openIntakeClaw()
//                        //RobotCommands.IntakeCommand(()->gamepad1.left_trigger))


        rightTrigger.whenActive(
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.down)

                //robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.up)
                //RobotCommands.IntakeDoneCommand()
                );

    }
    @Override
    public void run() {
        super.run();
        telemetry.addData("Joystick Position: ", robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        telemetry.update();
    }

    }
