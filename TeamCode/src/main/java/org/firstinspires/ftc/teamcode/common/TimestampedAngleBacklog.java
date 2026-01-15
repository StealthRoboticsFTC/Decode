package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

public class TimestampedAngleBacklog {
    private static class TimestampedAngle {
        private final long nanos;
        private final double angle;

        public TimestampedAngle(long nanos, double angle) {
            this.nanos = nanos;
            this.angle = angle;
        }

        public long getNanos() {
            return nanos;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static int MAX_POSES = 100;

    private int head = 0;
    private int size = 0;
    private TimestampedAngle[] poses = new TimestampedAngle[MAX_POSES];


    public TimestampedAngleBacklog() {
    }

    private int convertIdx(int idx) {
        if (idx < 0) {
            idx += size;
        }
        return (head + idx) % MAX_POSES;
    }

    public void addAngle(double angle) {
        if (size < MAX_POSES) {
            size++;
        } else {
            head++;
        }
        poses[convertIdx(-1)] = new TimestampedAngle(System.nanoTime(), angle);
    }

    public double getAngle(long nanos) {
        // ref: https://en.wikipedia.org/wiki/Binary_search_algorithm#Alternative_procedure
        int lo = 0;
        int hi = size - 1;
        while (lo != hi) {
            // equivalent of ceil division
            int mid = (lo + hi + 1) / 2;
            if (poses[convertIdx(mid)].getNanos() > nanos) {
                hi = mid - 1;
            } else {
                lo = mid;
            }
        }

        if (lo == size - 1) {
            return poses[convertIdx(lo)].getAngle();
        }

        TimestampedAngle angleLo = poses[convertIdx(lo)];
        TimestampedAngle angleHi = poses[convertIdx(lo + 1)];
        long tLo = angleLo.getNanos();
        long tHi = angleHi.getNanos();

        double t = ((double) (nanos - tLo)) / (tHi - tLo);

        double aLo = angleLo.getAngle();
        double aHi = angleHi.getAngle();

        return aLo + (aHi - aLo) * t;
    }
}