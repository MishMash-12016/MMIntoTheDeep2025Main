
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMDistSensor;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.Configuration;
import org.firstinspires.ftc.teamcode.utils.OpModeType;


/**
 * this class should contain all ur robot's attributes and systems
 */
public class MMSystems {


    public CuttleDigital elevatorSwitch;
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
    public Pose2d currentPose;

    //Subsystems
    public PinpointDrive driveTrain;
    public LinearIntake linearIntake;
    public IntakEndUnit intakEndUnit;
    public IntakeArm intakeArm;
    public IntakeEndUnitRotator intakeEndUnitRotator;
    public ScoringEndUnitElbow scoringEndUnitElbow;
    public ScoringArm scoringArm;
    public ScoringClawEndUnit scoringClawEndUnit;
    public Elevator elevator;

    public Vision vision;
    public DigitalChannel touchSensorScoring;
    public DigitalChannel touchSensorIntake;
    public DigitalChannel touchSensorIntakeHigh;
    public Climber climber;
    public static Pose2d AutoPose;


    public double servoDegrees = 0;


    //creating and initiating all subsystems
    public void initRobotSystems(MMOpMode mmOpMode) {
        this.scoringClawEndUnit = new ScoringClawEndUnit();
        this.elevator = new Elevator();
        this.linearIntake = new LinearIntake();
        this.intakEndUnit = new IntakEndUnit();
        this.intakeArm = new IntakeArm();
        this.scoringArm = new ScoringArm();
        this.intakeEndUnitRotator = new IntakeEndUnitRotator();
        this.elevatorSwitch = new CuttleDigital(MMRobot.getInstance().mmSystems.expansionHub, Configuration.elevatorTouchSensor);
        this.scoringEndUnitElbow = new ScoringEndUnitElbow();
        vision = new Vision(hardwareMap, telemetry);
        if (!limelightInitFunc(mmOpMode)){
            telemetry.addData("Oh no very sad no LIMELIGHT ):", "RESTART THE FUCKING ROBOT YOU WHORE");
        }
        this.touchSensorScoring = hardwareMap.get(DigitalChannel.class, "tsS");
        this.touchSensorIntake = hardwareMap.get(DigitalChannel.class, "tsIL");
        this.touchSensorIntakeHigh = hardwareMap.get(DigitalChannel.class, "tsIH");
        this.climber = new Climber();
    }

    public void initRobotSystemsTeleOp(MMOpMode mmOpMode) {
        this.scoringClawEndUnit = new ScoringClawEndUnit();
        this.elevator = new Elevator();
        this.linearIntake = new LinearIntake();
        this.intakEndUnit = new IntakEndUnit();
        this.intakeArm = new IntakeArm(false);
        this.scoringArm = new ScoringArm();
        this.intakeEndUnitRotator = new IntakeEndUnitRotator();
        this.elevatorSwitch = new CuttleDigital(MMRobot.getInstance().mmSystems.expansionHub, Configuration.elevatorTouchSensor);
        this.scoringEndUnitElbow = new ScoringEndUnitElbow();
        vision = new Vision(hardwareMap, telemetry);
        if (!limelightInitFunc(mmOpMode)){
            telemetry.addData("Oh no very sad no LIMELIGHT ):", "RESTART THE FUCKING ROBOT YOU WHORE");
        }
        this.touchSensorScoring = hardwareMap.get(DigitalChannel.class, "tsS");
        this.touchSensorIntake = hardwareMap.get(DigitalChannel.class, "tsIL");
        this.touchSensorIntakeHigh = hardwareMap.get(DigitalChannel.class, "tsIH");
        this.climber = new Climber();
    }


    public void initDriveTrain(Pose2d currentPose) {
        //roadRunner 90 is what we agree as 0 so reset it to 0
        localizer.setPosition(new Pose2d(0, 0, localizer.getHeading() - Math.toRadians(90)));
        driveTrain = new PinpointDrive(hardwareMap, currentPose);
    }

    public void initDriveTrain() {
        //roadRunner 90 is what we agree as 0 so reset it to 0
        localizer.setPosition(new Pose2d(0, 0, localizer.getHeading() - Math.toRadians(90)));
        driveTrain = new PinpointDrive(hardwareMap, currentPose);
    }

    public void teleop() {
        driveTrain.setDefaultCommand(
                MMRobot.getInstance().mmSystems.driveTrain.fieldOrientedDrive(
                        () -> Math.pow(gamepadEx1.getLeftX(), 3),
                        () -> Math.pow(gamepadEx1.getLeftY(), 3),
                        () -> Math.pow(gamepadEx1.getRightX(), 3)));
    }


    public boolean limelightInitFunc(MMOpMode mmOpMode) {
        ElapsedTime elapsedTime = new ElapsedTime();
        ElapsedTime elapsedTimeAll = new ElapsedTime();

        while (!MMRobot.getInstance().mmSystems.vision.switchToDetector() && !mmOpMode.isStopRequested()) {
            if (elapsedTimeAll.seconds() > 5) {
                return false;
            }
        }
        while (MMRobot.getInstance().mmSystems.vision.getPipelineIndex() != Vision.currentPipeline && !mmOpMode.isStopRequested()) {
            if (elapsedTimeAll.seconds() > 5) {
                return false;
            }
        }
        MMRobot.getInstance().mmSystems.vision.setPreviousResult();


        FtcDashboard.getInstance().getTelemetry().addData("started angle", elapsedTime.milliseconds());
        elapsedTime.reset();

        vision.findClosestForPython();
        FtcDashboard.getInstance().getTelemetry().addData("findClosestForPython", elapsedTime.milliseconds());
        elapsedTime.reset();


        while (!vision.switchToPython() && !mmOpMode.isStopRequested()) {
            if (elapsedTimeAll.seconds() > 5) {
                return false;
            }
        }
        FtcDashboard.getInstance().getTelemetry().addData("switchToPython", elapsedTime.milliseconds());
        elapsedTime.reset();


        while (vision.camera.getStatus().getPipelineIndex() != Vision.currentPipeline && !mmOpMode.isStopRequested()) {
            if (elapsedTimeAll.seconds() > 5) {
                return false;
            }
        }
        FtcDashboard.getInstance().getTelemetry().addData("switched to python time", elapsedTime.milliseconds());
        elapsedTime.reset();


        vision.camera.updatePythonInputs(new double[]{0.0, 0, 0, Vision.length, Vision.height, Vision.x, Vision.y, 0.0});

        FtcDashboard.getInstance().getTelemetry().addData("updated pyhton input", elapsedTime.milliseconds());
        elapsedTime.reset();
        vision.getTurnServoDegree();
//                limelightGetter.getRotateToSample(;
        FtcDashboard.getInstance().getTelemetry().addData("finished angle", elapsedTime.milliseconds());
        FtcDashboard.getInstance().getTelemetry().update();

        while (!vision.switchToDetector() && !mmOpMode.isStopRequested()) {
            if (elapsedTimeAll.seconds() > 5) {
                return false;
            }
        }
        while (!vision.switchToPython() && !mmOpMode.isStopRequested()) {
            if (elapsedTimeAll.seconds() > 5) {
                return false;
            }
        }
        while (!vision.switchToDetector() && !mmOpMode.isStopRequested()) {
            if (elapsedTimeAll.seconds() > 5) {
                return false;
            }
        }
        return true;
    }

    public void auto() {
        driveTrain.setDefaultCommand(new InstantCommand(() -> {
        }));
    }

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
        if (!hasImuBeenReset) {
            hasImuBeenReset = true;
            localizer = hardwareMap.get(GoBildaPinpointDriverRR.class, "imu");
            localizer.resetPosAndIMU();
            localizer.setOffsets(-99, 9);
            localizer.setEncoderResolution(GoBildaPinpointDriverRR.goBILDA_4_BAR_POD);
            localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            localizer.setPosition(new Pose2d(0, 0, Math.toRadians(90)));
        }
        currentPose = new Pose2d(0, 0, Math.toRadians(0));


        CommandScheduler.getInstance().reset(); //reset the scheduler
    }
}
