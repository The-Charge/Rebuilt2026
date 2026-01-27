package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.opencv.core.Mat.Tuple2;

public class PowerAngleOptimization {
    double[][] distances; // [angleIndex][velIndex]

    double[] angles;
    double[] velocities;

    public PowerAngleOptimization(double[][] distances, double[] angles, double[] velocities) {
        this.distances = distances;
        this.angles = angles;
        this.velocities = velocities;

        if (angles.length != distances.length || velocities.length != distances[0].length) {
            throw new Error("There are the wrong number of axis labels");
        }
    }

    public VelocityAngle optimize(double distance) {
        ArrayList<Tuple2<Integer>> betweenVelIndices = new ArrayList<>();

        for (int anglei = 0; anglei < angles.length; anglei++) {
            for (int veli  = 0; veli  < velocities.length - 1; veli ++) {
                if (between(distance, distances[anglei][veli], distances[anglei][veli+1])) {
                    betweenVelIndices.add(new Tuple2<Integer>(anglei, veli));
                    break;
                }
            }
        }

        ArrayList<Tuple2<Integer>> betweenAngleIndices = new ArrayList<>();

        for (int veli = 0; veli < velocities.length; veli++) {
            for (int anglei  = 0; anglei  < angles.length - 1; anglei ++) {
                if (between(distance, distances[anglei][veli], distances[anglei+1][veli])) {
                    betweenAngleIndices.add(new Tuple2<Integer>(anglei, veli));
                    break;
                }
            }
        }

        if (betweenAngleIndices.isEmpty() || betweenVelIndices.isEmpty()) {
            throw new Error("TODO: deal with case when distance is out of bounds");
        }

        double minimumDiff = Double.POSITIVE_INFINITY;
        VelocityAngle minimumDiffOutput = null;

        for (Tuple2<Integer> betweenAngleIndex : betweenAngleIndices) {
            int anglei = betweenAngleIndex.get_0();
            int veli = betweenAngleIndex.get_1();
            
            double distanceA = distances[anglei][veli];
            double distanceB = distances[anglei+1][veli];

            double diff = Math.abs(distanceA - distanceB);

            if (diff < minimumDiff) {
                minimumDiff = diff;
                double t = ( distance - distanceA)/(distanceB - distanceA) ; // this could be wrong mathematically
                minimumDiffOutput = new VelocityAngle(velocities[veli], lerp(t, angles[anglei], angles[anglei+1]));
            }
        }

        for (Tuple2<Integer> betweenVelIndex : betweenVelIndices) {
            int anglei = betweenVelIndex.get_0();
            int veli = betweenVelIndex.get_1();
            
            double distanceA = distances[anglei][veli];
            double distanceB = distances[anglei][veli+1];

            double diff = Math.abs(distanceA - distanceB);

            if (diff < minimumDiff) {
                minimumDiff = diff;
                double t = ( distance - distanceA)/(distanceB - distanceA) ; // this could be wrong mathematically
                minimumDiffOutput = new VelocityAngle(lerp(t, velocities[veli], velocities[veli+1]), angles[anglei]);
            }
        }

        return minimumDiffOutput;
    }

    double lerp(double t, double a, double b) {
        return a*(1-t) + b*t;
    }

    public boolean between(double min, double max, double x) {
        return min < x && x <= max;
    }

    public double[][] transpose(double[][] matrix) {
        int rows = matrix.length;
        int cols = matrix[0].length;

        double[][] t = new double[cols][rows];

        for (int r = 0; r < rows; r++) {
            for (int c  = 0; c  < cols; c ++) {
                t[c][r] = matrix[r][c];
            }
        }
        
        return t;
    }
}
