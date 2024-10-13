package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMRange;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;
import org.firstinspires.ftc.teamcode.MMRobot;

public class LinearIntakeCommand extends CommandBase{
    Trigger trigger;
    private final double TRIG_INPUT_START = 0;
    private final double TRIG_INPUT_END = 1;
    private final double TRIG_OUTPUT_START = 0;
    private final double TRIG_OUTPUT_END = 0.85;

    public LinearIntakeCommand(Trigger trigger){
        this.trigger = trigger;
        addRequirements(MMRobot.getInstance().mmSystems.linearIntake);
    }

    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(
                MMUtils.mapValuesLinearByRange(
                        MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                        new MMRange(TRIG_INPUT_START, TRIG_INPUT_END),
                        new MMRange(TRIG_OUTPUT_START, TRIG_OUTPUT_END)
                )
        );
    }

    @Override
    public boolean isFinished() {
//        return false;
        return !trigger.get();
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);
    }
}
