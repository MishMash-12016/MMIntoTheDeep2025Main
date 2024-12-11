package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class IntakeTeleOp extends MMOpMode {
    MMRobot robotInstance;

    double currentPose = 0 ;

    public IntakeTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Trigger leftTrigger = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
        );

        Trigger rightTrigger = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
        );

        leftTrigger.whileActiveOnce(
                new SequentialCommandGroup(
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.down),
                robotInstance.mmSystems.intakEndUnit.openIntakeClaw()

                //RobotCommands.IntakeCommand(()->gamepad1.left_trigger)
        ));

        leftTrigger.whenInactive(
                new SequentialCommandGroup(
                robotInstance.mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(300),
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.up),
                robotInstance.mmSystems.linearIntakeEndUnitRotator.setPosition(LinearIntakeEndUnitRotator.holdpose)

                //RobotCommands.IntakeDoneCommand()
        ));

        rightTrigger.whileActiveOnce(
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.down)

        );

//        Trigger addToPoseCondition = new Trigger(
//                () -> robotInstance.mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
//        );
//        Trigger subToPoseCondition = new Trigger(
//                () -> robotInstance.mmSystems.gamepadEx1.getButton(GamepadKeys.Button.B)
//        );
//        Trigger littleAddToPoseCondition = new Trigger(
//                () -> robotInstance.mmSystems.gamepadEx1.getButton(GamepadKeys.Button.X)
//        );
//        Trigger littleSubToPoseCondition = new Trigger(
//                () -> robotInstance.mmSystems.gamepadEx1.getButton(GamepadKeys.Button.Y)
//        );
//        addToPoseCondition.whenActive(() ->currentPose+=0.05);
//        subToPoseCondition.whenActive(() ->currentPose-=0.05);
//        littleAddToPoseCondition.whenActive(() ->currentPose+=0.01);
//        littleSubToPoseCondition.whenActive(() ->currentPose-=0.01);

//        robotInstance.mmSystems.intakeArm.setDefaultCommand(robotInstance.mmSystems.intakeArm.setPosition(()->currentPose));

//        rightTriggerCondition.whenActive(
//                robotInstance.mmSystems.intakeArm.setPosition(currentPose));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Joystick Position: ", robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        telemetry.addData("current pose: ", currentPose);
        telemetry.update();
    }

}