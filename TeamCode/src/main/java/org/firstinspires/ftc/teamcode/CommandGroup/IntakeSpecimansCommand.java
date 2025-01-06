package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class IntakeSpecimansCommand {
    public static Command PrepareSpecimanIntake(){
        return new SequentialCommandGroup(
            MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.midPose), //be prepared for transfer
            MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
            MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.specimanIntake),
            MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.intakeSpecimanPose),
            MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw());
    }
    public static Command SpecimenIntake(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.specimanIntake),//make sure your their
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.transferPose),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.holdposespeciman),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.closedPose), //makes sure
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(), //makes sure
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.midPose)


        );
    }
}
