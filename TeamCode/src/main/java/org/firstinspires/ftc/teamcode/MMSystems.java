
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMDistSensor;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.SubSystems.Wisher;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.Configuration;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

/**
 * this class should contain all ur robot's attributes and systems
 */
public class MMSystems {


    public  CuttleDigital elevatorSwitch;
    //Attributes & Hardware
    public OpModeType opModeType;
    public AllianceColor allianceColor;
    public AllianceSide robotSide;
    public HardwareMap hardwareMap;
    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    public Telemetry telemetry;
    public MMBattery battery;
    public MMDistSensor intakeDistSensor;

    public static GoBildaPinpointDriverRR localizer;
    static boolean hasImuBeenReset = false;
    public Pose2d localizerCurrentPose;


    //Subsystems
    public DriveTrain driveTrain;
    public LinearIntake linearIntake;
    public IntakEndUnit intakEndUnit;
    public IntakeArm intakeArm;
    public IntakeEndUnitRotator intakeEndUnitRotator;
    public ScoringEndUnitRotator scoringEndUnitRotator;
    public ScoringArm scoringArm;
    public ScoringClawEndUnit scoringClawEndUnit;
    public Elevator elevator;
    public ScoringEndUnitElbow scoringEndUnitElbow;
    public Wisher wisher;

    public Vision vision;

    public enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    public Mode currentMode = Mode.DRIVER_CONTROL;

    public SequentialCommandGroup currentAutoActions;



    //creating and initiating all subsystems
    public void initRobotSystems() {

        this.elevator = new Elevator(elevatorSwitch);
        this.linearIntake = new LinearIntake();
        this.intakEndUnit = new IntakEndUnit();
        this.scoringEndUnitRotator = new ScoringEndUnitRotator();
        this.intakeArm = new IntakeArm();
        this.scoringArm = new ScoringArm();
        this.scoringClawEndUnit = new ScoringClawEndUnit();
        this.intakeEndUnitRotator = new IntakeEndUnitRotator();
        this.elevatorSwitch = new CuttleDigital(MMRobot.getInstance().mmSystems.expansionHub, Configuration.elevatorTouchSensor);
        this.scoringEndUnitElbow = new ScoringEndUnitElbow();
        this.wisher = new Wisher();
        vision = new Vision(hardwareMap, telemetry);
//        linearIntake.setDefaultCommand(
//                linearIntake.defultCommand(0)
//        );

    }

    public void initDriveTrain() {
        //roadRunner 90 is what we agree as 0 so reset it to 0
        localizer.setPosition(new Pose2d(0, 0, localizer.getHeading() - Math.toRadians(90)));
        driveTrain = new DriveTrain();
    }

    public SequentialCommandGroup joystickDrive(){
        if (currentMode == Mode.DRIVER_CONTROL) {
            return new SequentialCommandGroup(new InstantCommand(()->
                    MMRobot.getInstance().mmSystems.driveTrain.fieldOrientedDrive(
                            () -> Math.pow(gamepadEx1.getLeftX(), 3),
                            () -> Math.pow(gamepadEx1.getLeftY(), 3),
                            () -> Math.pow(gamepadEx1.getRightX(), 3))

            ));
        }
        else if (currentMode == Mode.AUTOMATIC_CONTROL) {
            if (gamepadEx2.gamepad.a)
            {
                return currentAutoActions;
            }
        }
        return new SequentialCommandGroup(new InstantCommand(()->MMRobot.getInstance().mmSystems.telemetry.addData("wtf",0)));
    };

    public MMSystems(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.opModeType = type;
        this.hardwareMap = hardwareMap;
        this.controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);
        if (type != OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION) {
            this.expansionHub = new CuttleRevHub(hardwareMap, "Expansion Hub 1");
        }
        this.gamepadEx1 = new GamepadEx(gamepad1);
        this.gamepadEx2 = new GamepadEx(gamepad2);
        this.telemetry = telemetry;
        this.battery = new MMBattery(hardwareMap);
        this.intakeDistSensor = new MMDistSensor(hardwareMap);
        if(!hasImuBeenReset){
            hasImuBeenReset = true;
            localizer = hardwareMap.get(GoBildaPinpointDriverRR.class,"imu");
            localizer.resetPosAndIMU();
            localizer.setOffsets(-99, 9);
            localizer.setEncoderResolution(GoBildaPinpointDriverRR.goBILDA_4_BAR_POD);
            localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            localizer.setPosition(new Pose2d(0,0,Math.toRadians(90)));
        }



        CommandScheduler.getInstance().reset(); //reset the scheduler
    }
}
