package org.firstinspires.ftc.teamcode.utils;

import org.opencv.core.Mat;

import java.util.List;

public class MathTools {
    public static double distance(List<Double> p1, List<Double> p2){
        return Math.sqrt((p1.get(0) - p2.get(0)) * (p1.get(0) - p2.get(0)) + (p1.get(1) - p2.get(1)) * (p1.get(1) - p2.get(1)));
    }

    public static double distance(double p1, double p2){
        return Math.sqrt((p1 - p2) * (p1 - p2));
    }
}
