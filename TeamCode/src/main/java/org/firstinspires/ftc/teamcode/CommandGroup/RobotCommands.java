package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

import java.util.function.DoubleSupplier;

public class RobotCommands {

    private static final MMSystems mmSystems = MMRobot.getInstance().mmSystems;
    private static final double linearIntakeClosed = 0;

    /* intake recieve -
    1. open linear intake and
    3. intake down and open claw*/

    public static Command IntakeCommand(DoubleSupplier intake_trigger) {
        return new ParallelCommandGroup(
                mmSystems.linearIntake.setPositionByJoystick(intake_trigger),
                        mmSystems.intakeArm.intakeDown(),
                        mmSystems.intakEndUnit.openIntakeClaw());


    }

    /* intake done  -
    1. close claw
    2. intake arm up  ,
    3. close linear intake */
    public static Command IntakeDoneCommand() {
        return new SequentialCommandGroup(
                mmSystems.intakEndUnit.closeIntakeClaw(),
                mmSystems.intakeArm.intakeUp()).
                andThen(mmSystems.linearIntake.setPosition(linearIntakeClosed));
    }

    /* score sample-
    1. elevator get to desired height
    2. scoring servo turn
    3. scoring claw open
     */
    public static Command PrepareHighSample() {
        return new ParallelCommandGroup(
                mmSystems.elevator.moveToPose(Elevator.HIGH_BASKET),
                mmSystems.scoringEndUnit.scoreScoringServo())
                ;
    }

    /* scoring back to hold-
    1. scoring claw close
    2. elevator go back to desired height and scoring servo turn
     */
    public static Command DefaultPosScoring() {
        return new SequentialCommandGroup(mmSystems.scoringEndUnit.closeScoringClaw(),
                new ParallelCommandGroup(
                        mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                        mmSystems.scoringEndUnit.holdScoringServo()));
    }
}

