package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

public class IntakeArm extends SubsystemBase {
    CuttleServo servoLeft;
    CuttleServo servoRight;

    public final static double up = 0.2;
    public final static double down = 0.3;
    public final static double midpose =0.8;

    public IntakeArm() {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.INTAKE_ARM_SERVO_LEFT);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.INTAKE_ARM_SERVO_RIGHT);
    }

    //tell servo intake to get to down position
    public Command setPosition(double newPos) {
        return new InstantCommand(()-> {
            servoLeft.setPosition(newPos);
            servoRight.setPosition(1-newPos);} ,
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
