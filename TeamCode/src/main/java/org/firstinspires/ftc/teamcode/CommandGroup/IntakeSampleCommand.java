package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class IntakeSampleCommand {

    public static Command prepareSampleIntake(double trigger){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(trigger),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.prepareSampleIntake),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
        );
    }
    public static Command SampleIntake(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.intakePose), //collect pose
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.transferPose),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.closedPose),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
        );
    }  public static Command sampleEject(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.prepareSampleIntake), //mid pose
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.transferPose)
        );
    }

}
