package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.StartEndCommand;

import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.RollerIntake;

public class IntakeArmDownAndStartRoller extends StartEndCommand {
    public IntakeArmDownAndStartRoller() {
        super(
                () -> {
                    MMRobot.getInstance().mmSystems.rollerIntake.setPower(RollerIntake.ON);
                    new IntakeArmSetState(IntakeArm.Position.OUT).schedule();
                },
                () -> {
                    MMRobot.getInstance().mmSystems.rollerIntake.setPower(RollerIntake.OFF);
                    new IntakeArmSetState(IntakeArm.Position.IN).schedule();
                },
                MMRobot.getInstance().mmSystems.rollerIntake,MMRobot.getInstance().mmSystems.intakeArm
        );
    }
}
