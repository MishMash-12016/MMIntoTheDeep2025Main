package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

public class TeleopGeneral extends MMOpMode {
    //Constructor for TeleopGeneral.
    public TeleopGeneral() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    /**
     * Initializes the robot systems and sets up button triggers for specific actions.
     */
    public static boolean specimantoggled = true;
    public static boolean eleavatortoggled = true;

    @Override
    public void onInit() {
        //init robot systems
        MMRobot.getInstance().mmSystems.initRobotSystems();


        //define robot triggers
        //driver 1
        Trigger intakeTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05);
        Trigger ejectTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
        );

        //driver 2
        Trigger prepareHighSampleTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.X)
        );
        Trigger prepareLowSampleTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.A)
        );

        Trigger scoreHigheSampleTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.Y)
        );
        Trigger specimanIntakeTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.B)
        );

        Trigger prepareIntakeAngle = new Trigger( //command to change the intake servo
                () -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        );
        Trigger specimanScoreTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        );
        //define actions for triggers

        // Execute scoring (Button Y)
        scoreHigheSampleTrigger.whileActiveOnce(
                MMRobot.getInstance().mmSystems.scoringEndUnit.openScoringClaw()
        );

        //driver 1
        // Handle intake (Right Trigger)
        intakeTrigger.whileActiveOnce(
                RobotCommands.IntakeCommand(
                        () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)

                ).whenFinished(
                        () -> RobotCommands.IntakeDoneCommand().schedule()));
        //eject sample (Button A)
        ejectTrigger.whileActiveOnce(
                RobotCommands.EjectSampleCommand()
        );

        //driver 2

        // Prepare to score High Basket(Button X)
        prepareHighSampleTrigger.whileActiveOnce(
                RobotCommands.PrepareHighSample()
        );
        // Prepare to score Low Basket(Button A)
        prepareLowSampleTrigger.whileActiveOnce(
                RobotCommands.PrepareLowSample()
        );
        //score speciman (right bumper)
        specimanScoreTrigger.whileActiveOnce(new RunCommand(() -> {
            if (eleavatortoggled) {
                RobotCommands.elevatorDowm();
                eleavatortoggled = false;
            } else {
                RobotCommands.ScoreSpeciman();
                eleavatortoggled = true;

            }
        }));

        //joystick prepare angle of intake

        //intake speciman (Button B)
        specimanIntakeTrigger.whileActiveOnce(new RunCommand(() -> { //run command cause gpt told me to
            if (specimantoggled) {
                RobotCommands.SpecimanIntake();
                specimantoggled = false;
            } else {
                RobotCommands.PrepareSpecimanScore();
                specimantoggled = true;
            }
        }));

        //hanging joystick

    }

    @Override
    public void run() {
        super.run();
    }
}
