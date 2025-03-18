package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class OneandFive {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double maxWheelVel = 70;
        double maxProfileAccel = 70;
        final double tangentsToScoreSpecimen = 135;
        final double tangentsToIntakeSpecimen = 310;
        final Pose2d intakePose = new Pose2d(38, -66, Math.toRadians(90));
        final Pose2d scorePose = new Pose2d(5, -33, Math.toRadians(120));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)//+(150/25.4)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(270)))

//                        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(2, -28), Math.toRadians(90))

//        TrajectoryActionBuilder driveToEject = driveToScorePreload.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(0))

//        TrajectoryActionBuilder driveToPush1 = driveToEject.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(29, -35, Math.toRadians(225)), Math.toRadians(50))
//        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(35.8, -51, Math.toRadians(150)), Math.toRadians(250))

//        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70))
//        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -51, Math.toRadians(150)), Math.toRadians(260))

//        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80))
//        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(51, -53, Math.toRadians(90)), Math.toRadians(270))

//        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(51,-66, Math.toRadians(90)), Math.toRadians(270))
//        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140+5))
                .splineToLinearHeading(new Pose2d(10,-34,Math.toRadians(112.5)), Math.toRadians(tangentsToScoreSpecimen))
//        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen))
//        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen))
//        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen))
//        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen))
//        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen))
//        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen))
//        TrajectoryActionBuilder driveToIntakeFifthSpecimen = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen))
//        TrajectoryActionBuilder driveToScoreFifthSpecimen = driveToIntakeFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen))
//        TrajectoryActionBuilder driveToPark = driveToScoreFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                        .splineToLinearHeading(new Pose2d(24, -50, Math.toRadians(140)), Math.toRadians(140+180))


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


