package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class IntakeArm extends SubsystemBase {
    CuttleServo servoLeft;
    CuttleServo servoRight;
    public enum IntakeArmState {
        INTAKE_POSE(0.6),
        PREPARE_SAMPLE_INTAKE(0.5),
        SPECIMEN_INTAKE(0.34),
        MID_INTAKE_SPECIMEN(0.3),
        TRANSFER_POSE(0.04);
        public double position;
        IntakeArmState(double position){
            this.position = position;
        }}

    public IntakeArm() {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_LEFT);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_RIGHT);
        servoLeft.setPosition(IntakeArmState.TRANSFER_POSE.position);
        servoRight.setPosition(1-IntakeArmState.TRANSFER_POSE.position);
    }

    //tell servo intake to get to down position
    public Command setPosition(double newPos) {
        return new InstantCommand(()-> {
            servoLeft.setPosition(newPos);
            servoRight.setPosition(1-newPos);} ,
                this);
    }
    public Command setPosition(IntakeArmState state) {
        return new InstantCommand(()-> {
            servoLeft.setPosition(state.position);
            servoRight.setPosition(1-state.position);} ,
                this);
    }


//    //tell servo intake to get to down position
//    public Command setPosition(DoubleSupplier newPos) {
//        return new RunCommand(()-> {
//            servoLeft.setPosition(newPos.getAsDouble());
//            servoRight.setPosition(1-newPos.getAsDouble());} ,
//                this);
//    }
}
