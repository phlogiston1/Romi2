package frc.lib.math;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class LineMath {
    
    public static LineEquation getBisectorOfPoints(Point2d point1, Point2d point2){
        Line bsline = new Line();
        bsline.x1 = point1.getX();
        bsline.y1 = point1.getY();
        bsline.x2 = point2.getX();
        bsline.y2 = point2.getY();

        LineEquation bisectorEquation = new LineEquation();
        bisectorEquation.slope = -1/bsline.getSlope();

        double offset = bsline.getMidpoint().getY() - bisectorEquation.calc(bsline.getMidpoint().getX());
        bisectorEquation.offset = offset;
    
        return bisectorEquation;
    }

    public static double distanceBetween(Point2d p1, Point2d p2){
        //sqrt((x2-x1)^2 + (y2 - y1)^2)
        return Math.sqrt((p2.getX() - p1.getX())*(p2.getX() - p1.getX()) + (p2.getY() - p1.getY())*(p2.getY() - p1.getY()));
    }

    public static Point2d getIntersectionOfLines(Line l1, Line l2){
        LineEquation l1eq = l1.getEquation();
        LineEquation l2eq = l2.getEquation();
        double xm =  l1eq.slope - l2eq.slope;
        double eq = l2eq.offset - l1eq.offset;
        double x = eq / xm;
        double y = l1eq.calc(x);
        return new Point2d(x,y);
    }

    public static Rotation2d angleBetween(Line l1, Line l2){
        return l1.getAngleOfInclination().minus(l2.getAngleOfInclination());
    }
    
    //testing:
    public static void main(String args[]){
        Line ln = new Line();
        ln.x1 = 0;
        ln.y1 = 1;
        ln.x2 = 1;
        ln.y2 = 0;
        Line ln2 = new Line();
        ln2.x1 = 0;
        ln2.y1 = 0;
        ln2.x2 = 1;
        ln2.y2 = 1;
        System.out.println(angleBetween(ln, ln2).toString());
        Line test = Line.fromPose(new Pose2d(10,10,Rotation2d.fromDegrees(91)));
        Line test2 = Line.fromEquation(getBisectorOfPoints(new Point2d(10,10), new Point2d(12,12)));
        test.getEquation().print();
        test2.getEquation().print();
        System.out.println(getIntersectionOfLines(test, test2).getY()); 
        System.out.println(getIntersectionOfLines(test,test2).getX()); 
        // getBisectorOfPoints(new Point2d(1,1), new Point2d(2, 3)).print();
    }



    public static class Line{
        public double x1,y1,x2,y2;
        public double getSlope(){
            return (y2-y1)/(x2-x1);
        }
        public LineEquation getEquation(){
            LineEquation eq = new LineEquation();
            eq.slope = getSlope();
            double offset = y1 - eq.calc(x1);
            eq.offset = offset;
            return eq;
        }
        public Rotation2d getAngleOfInclination(){
            LineEquation eq = getEquation();
            double slope = eq.slope;
            double inc = Math.atan(slope);
            return new Rotation2d(inc);
        }
        public void assign(double X1, double Y1, double X2, double Y2){
            x1 = X1;
            y1 = Y1;
            x2 = X2;
            y2 = Y2;
        }
        public void assign(Line newLine){
            assign(newLine.x1,newLine.y1,newLine.x2,newLine.y2);
        }
        public Point2d getMidpoint(){
            double x = x1 + (x2 - x1)/2;
            double y = y1 + (y2 - y1)/2;
            return new Point2d(x,y);
        }
        public static Line fromPose(Pose2d pose){
            PolarPoint2d point = new PolarPoint2d(0,pose.getRotation());
            Line result = new Line();
            result.x1 = pose.getX();
            result.y1 = pose.getY();
            point.transformBy(10, 0);
            Point2d cart = PolarPoint2d.getCartesianPoint(point);
            result.x2 = pose.getX() + cart.getX();
            result.y2 = pose.getY() + cart.getY();
            return result;
        }
        public void slopeInterceptTransform(double slope, double intercept){
            LineEquation eq = getEquation();
            eq.slope += slope;
            eq.offset += intercept;
            Line newLine = fromEquation(eq);
            assign(newLine);
        }
        public void print(){
            System.out.println("x1 "+x1+" y1 "+y1+" x2 "+x2+" y2 " + y2);
        }
        public static Line fromEquation(LineEquation eq){
            Line out = new Line();
            out.x1  = 0;
            out.y1 = eq.calc(0);
            out.x2 = 1;
            out.y2 = eq.calc(1);
            return out;
        }
    } 

    public static class LineEquation{
        public double slope, offset;
        public double calc(double x) {
            return (x*slope) + offset;
        }
        public void print(){
            System.out.println("slope: " + slope + " offset: " + offset);
        }
    }
}
