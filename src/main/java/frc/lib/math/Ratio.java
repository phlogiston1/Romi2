package frc.lib.math;

public class Ratio {
    public double n1 = 1, n2 = 1;
    public void calculate(double num1, double num2){
        if(num1 >= num2){
            n1 = 1;
            n2 = num2 / num1;
        } else {
            n2 = 1;
            n1 = num2 / num1;
        }
    }
    public String toString(){
        return "ratio: " + n1 + ":" + n2;
    }
}
