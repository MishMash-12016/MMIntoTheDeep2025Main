package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

@Config
public class Climber extends SubsystemBase {
    private final CRServo servoLeft;
    private final CRServo servoRight;



    public Climber(){
        servoLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(CRServo.class, "climber right");//3
        servoRight = MMRobot.getInstance().mmSystems.hardwareMap.get(CRServo.class, "climber left");//5

        setDefaultCommand(defultCommand(0));
    }

    public Command setPower(double newPower){
        return new InstantCommand(()-> {
            servoRight.setPower(newPower);
            servoLeft.setPower(-newPower);} ,
                this);
    }

    public Command stop(){
        return new InstantCommand(()-> {
            servoRight.setPower(0);
            servoLeft.setPower(0);} ,
                this);
    }

    public Command defultCommand(double newPower){

        return new RunCommand(()-> {
            servoRight.setPower(newPower);
            servoLeft.setPower(-newPower);} ,
                this);
    }

}