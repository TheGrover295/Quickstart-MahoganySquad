package org.firstinspires.ftc.teamcode.pedroPathing.AutoRecord;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

public class CubicSpline1D {
    private double[] t, a, b, c, d;
    double endTime;

    public CubicSpline1D(List<TimePose> samples, Function<TimePose, Double> valueGetter) {
        int n = samples.size();
        if (n < 2) return;
        t = new double[n];
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            t[i] = samples.get(i).t;            // time
            y[i] = valueGetter.apply(samples.get(i)); // x, y, or theta
        }

        endTime = t[n-1];
        a = Arrays.copyOf(y, n);
        b = new double[n-1];
        c = new double[n];
        d = new double[n-1];

        double[] h = new double[n-1];
        for (int i = 0; i < n-1; i++) h[i] = t[i+1] - t[i];

        double[] alpha = new double[n];
        for (int i = 1; i < n-1; i++)
            alpha[i] = (3.0/h[i])*(a[i+1]-a[i]) - (3.0/h[i-1])*(a[i]-a[i-1]);

        double[] l = new double[n];
        double[] mu = new double[n];
        double[] z = new double[n];

        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;

        for (int i = 1; i < n-1; i++) {
            l[i] = 2*(t[i+1]-t[i-1]) - h[i-1]*mu[i-1];
            mu[i] = h[i]/l[i];
            z[i] = (alpha[i] - h[i-1]*z[i-1])/l[i];
        }

        l[n-1] = 1;
        z[n-1] = 0;
        c[n-1] = 0;

        for (int j = n-2; j >= 0; j--) {
            c[j] = z[j] - mu[j]*c[j+1];
            b[j] = (a[j+1]-a[j])/h[j] - h[j]*(c[j+1]+2*c[j])/3.0;
            d[j] = (c[j+1]-c[j])/(3.0*h[j]);
        }
    }

    public double evaluate(double tQuery) {
        if (t == null || t.length < 2) return 0;
        int i = findSegment(tQuery);
        double dt = tQuery - t[i];
        return a[i] + b[i]*dt + c[i]*dt*dt + d[i]*dt*dt*dt;
    }

    public double derivative(double tQuery) {
        if (t == null || t.length < 2) return 0;
        int i = findSegment(tQuery);
        double dt = tQuery - t[i];
        return b[i] + 2*c[i]*dt + 3*d[i]*dt*dt;
    }

    public double secondDerivative(double tQuery) {
        if (t == null || t.length < 2) return 0;
        int i = findSegment(tQuery);
        double dt = tQuery - t[i];
        return 2*c[i] + 6*d[i]*dt;
    }

    private int findSegment(double tQuery) {
        if (tQuery <= t[0]) return 0;
        if (tQuery >= t[t.length-1]) return t.length-2;
        for (int i = 0; i < t.length-1; i++) {
            if (tQuery >= t[i] && tQuery <= t[i+1]) return i;
        }
        return t.length-2;
    }

    public double endTime() {
        return endTime;
    }
}
